/*
 * CMSIS-DAP v1 command layer (HID, 64-byte reports) on top of swd_dap.c.
 * Layouts follow ARM DAPLink's DAP.c (Apache-2.0).
 */

#include "dap_hid.h"

#include <stdio.h>
#include <string.h>

#include "tusb.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "cw_isp_prog.h"
#include "cw_vdd.h"
#include "serial.h"
#include "swd_dap.h"

#ifdef USB_ITF_HID
#error "USB_ITF_HID and USB_ITF_DAP cannot both be enabled: ESP32-S3 has 5 IN endpoints"
#endif

#define DAP_PACKET 64

#define DAP_RSP_OK  0x00
#define DAP_RSP_ERR 0xFF

#define XFER_OK       0x01
#define XFER_WAIT     0x02
#define XFER_FAULT    0x04
#define XFER_ERROR    0x08
#define XFER_MISMATCH 0x10

#define DP_RDBUFF 0x0Cu
#define DP_ABORT  0x00u

enum {
    ID_DAP_INFO = 0x00,
    ID_DAP_HOST_STATUS = 0x01,
    ID_DAP_CONNECT = 0x02,
    ID_DAP_DISCONNECT = 0x03,
    ID_DAP_TRANSFER_CONFIGURE = 0x04,
    ID_DAP_TRANSFER = 0x05,
    ID_DAP_TRANSFER_BLOCK = 0x06,
    ID_DAP_TRANSFER_ABORT = 0x07,
    ID_DAP_WRITE_ABORT = 0x08,
    ID_DAP_DELAY = 0x09,
    ID_DAP_RESET_TARGET = 0x0A,
    ID_DAP_SWJ_PINS = 0x10,
    ID_DAP_SWJ_CLOCK = 0x11,
    ID_DAP_SWJ_SEQUENCE = 0x12,
    ID_DAP_SWD_CONFIGURE = 0x13,
    ID_DAP_SWD_SEQUENCE = 0x1D,
    ID_DAP_QUEUE_COMMANDS = 0x7E,
    ID_DAP_EXECUTE_COMMANDS = 0x7F,
    ID_DAP_INVALID = 0xFF,
};

enum {
    DAP_ID_VENDOR = 0x01,
    DAP_ID_PRODUCT = 0x02,
    DAP_ID_SER_NUM = 0x03,
    DAP_ID_DAP_FW_VER = 0x04,
    DAP_ID_DEVICE_VENDOR = 0x05,
    DAP_ID_DEVICE_NAME = 0x06,
    DAP_ID_BOARD_VENDOR = 0x07,
    DAP_ID_BOARD_NAME = 0x08,
    DAP_ID_PRODUCT_FW_VER = 0x09,
    DAP_ID_CAPABILITIES = 0xF0,
    DAP_ID_PACKET_COUNT = 0xFE,
    DAP_ID_PACKET_SIZE = 0xFF,
};

static uint8_t req_buf[DAP_PACKET];
static uint8_t rsp_buf[DAP_PACKET];
static bool rsp_pending;

static uint8_t debug_port;
static uint16_t retry_count = 100;
static uint16_t match_retry_count;
static uint32_t match_mask;
static bool transfer_abort;
static bool rail_owned;
static uint32_t xfer_errs;

static inline uint32_t get32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static inline void put32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static inline bool rsp_room(const uint8_t *head, const uint8_t *p, uint32_t n) {
    return (size_t)(p - head) + n <= DAP_PACKET;
}

static int rescue_count;

/* This DP latches STICKYORUN as soon as SWDIO is clocked high, and a line
 * reset does exactly that.  While STICKYORUN is set every DP write answers
 * FAULT, which is what kills the tool's power-up and AP writes right after
 * its own line reset.  ABORT clears it, so clear it after any sequence that
 * might have clocked SWDIO high. */
static void dap_dp_clear(void) {
    uint32_t abort = 0x1E;

    (void)swd_dap_transfer(DP_ABORT, &abort);
}

#if defined(CWW_DAP_ONLY)
/* Put the pads back into the SWD pinout.  Cheap and deterministic, so every
 * attach attempt starts from a known pad state. */
static void dap_repad(void) {
    swd_dap_release();
    swd_dap_claim();
}
#endif

#if defined(CWW_DAP_ONLY)
/* Connect in two steps.  The JTAG-to-SWD select is not idempotent on this
 * part: sent to a DP that is already in SWD mode it puts the DP back into
 * JTAG mode and every later request gets NOACK.  So an already-switched DP is
 * taken with a plain line reset and IDCODE read, and the select is only ever
 * sent when that probe fails (a fresh chip powers up in JTAG). */
static bool dap_ensure_swd(uint32_t *id_out) {
    uint32_t id = 0;

    swd_dap_line_reset();
    if (swd_dap_transfer(0x02, &id) != SWD_DAP_ACK_OK) {
        swd_dap_enter_swd();
        if (swd_dap_transfer(0x02, &id) != SWD_DAP_ACK_OK) {
            return false;
        }
    }
    /* The line reset that established SWD also latched STICKYORUN. */
    dap_dp_clear();
    if (id_out) {
        *id_out = id;
    }
    return true;
}
#endif

/* A DP that has lost the request framing stops answering, and after that a
 * plain line reset only buys a single answered request.  The JTAG-to-SWD
 * select puts it back for good, so recovery always goes through the select
 * and requires two answered reads before the session continues. */
static bool dap_recover_swd(void) {
    uint32_t id = 0;

    for (int attempt = 0; attempt < 2; attempt++) {
        swd_dap_line_reset();
        for (int i = 0; i < 8; i++) {
            if (swd_dap_transfer(0x02, &id) == SWD_DAP_ACK_OK) {
                break;
            }
        }
        if (swd_dap_transfer(0x02, &id) != SWD_DAP_ACK_OK) {
            swd_dap_enter_swd();
            if (swd_dap_transfer(0x02, &id) != SWD_DAP_ACK_OK) {
                vTaskDelay(1);
                continue;
            }
        }
        dap_dp_clear();
        return true;
    }
    return false;
}

#if defined(CWW_DAP_ONLY)
static bool dap_attach_under_reset(uint32_t *id_out) {
    uint32_t id = 0;
    uint32_t hz = swd_dap_clock();
    bool via_reset = false;

    swd_dap_set_clock(1000000);

    dap_repad();
    if (dap_ensure_swd(&id)) {
        printf("[dap] phase1 idcode ok\n");
        goto attached;
    }
    for (int poll = 0; poll < 200; poll++) {
        if (dap_ensure_swd(&id)) {
            printf("[dap] phase1 poll %d ok\n", poll);
            goto attached;
        }
    }

    /* The application may remap PA2/PB6 within microseconds of starting, so
     * release NRST and poll immediately: a few ms of delay is already too
     * late for that window. */
    swd_dap_take_nrst(true);
    via_reset = true;
    for (int attempt = 0; attempt < 4; attempt++) {
        dap_repad();
        swd_dap_nreset(0);
        swd_dap_delay_us(1500);
        swd_dap_nreset(1);
        swd_dap_enter_swd();
        for (int poll = 0; poll < 2000; poll++) {
            if (swd_dap_transfer(0x02, &id) == SWD_DAP_ACK_OK) {
                printf("[dap] idcode in reset attempt %d poll %d\n", attempt, poll);
                goto attached;
            }
        }
        vTaskDelay(1);
    }
    swd_dap_take_nrst(false);
    swd_dap_set_clock(hz);
    return false;

attached:
    swd_dap_set_clock(hz);
    printf("[dap] attached %s, id=%08lx\n", via_reset ? "after reset" : "live",
           (unsigned long)id);
    if (id_out) {
        *id_out = id;
    }
    return true;
}
#endif

/* Runs one SWD transfer, retrying WAIT responses like DAPLink does.
 * A missing target answers with ACK=0b111, which is not one of the
 * CMSIS-DAP transfer response codes; report it as DAP_TRANSFER_ERROR. */
static uint8_t swd_xfer_retry(uint8_t rq, uint32_t *data, uint32_t retry) {
    uint8_t ack;

    do {
        ack = swd_dap_transfer(rq, data);
    } while (ack == SWD_DAP_ACK_WAIT && retry-- && !transfer_abort);

    if (ack == SWD_DAP_ACK_OK) {
        rescue_count = 0;
    }
    else if (ack == SWD_DAP_ACK_FAULT && debug_port == 1 && rescue_count < 3) {
        /* A FAULT latches the DP error state and blocks every later write
         * until ABORT is written. */
        uint32_t abort = 0x1E;

        rescue_count++;
        printf("[dap] fault, aborting (of %lu)\n", (unsigned long)rq);
        if (swd_dap_transfer(DP_ABORT, &abort) == SWD_DAP_ACK_OK) {
            ack = swd_dap_transfer(rq, data);
        }
    }
    else if (ack != SWD_DAP_ACK_WAIT && debug_port == 1 && rescue_count < 3) {
        rescue_count++;
        printf("[dap] ack=%u, recovery (%d)\n", (unsigned)ack, rescue_count);
        if (dap_recover_swd()) {
            ack = swd_dap_transfer(rq, data);
        }
    }

    switch (ack) {
        case SWD_DAP_ACK_OK:
            return XFER_OK;
        case SWD_DAP_ACK_WAIT:
            return XFER_WAIT;
        case SWD_DAP_ACK_FAULT:
            return XFER_FAULT;
        default:
            if (xfer_errs < 6) {
                printf("[dap] xfer rq=%02X ack=%u\n", (unsigned)rq, (unsigned)ack);
                xfer_errs++;
            }
            return XFER_ERROR;
    }
}

static bool dap_ensure_pads(void) {
    if (!swd_dap_claimed()) {
        cw_isp_pads_hold_for_swd();
        swd_dap_claim();
    }
    return swd_dap_claimed();
}

/* Length of one DAP_Transfer item: request byte, plus 4-byte write data or
 * 4-byte match value. */
static const uint8_t *dap_item_end(const uint8_t *p, const uint8_t *limit) {
    uint32_t extra;

    if (p >= limit) {
        return limit;
    }
    if (*p & SWD_DAP_RNW) {
        extra = (*p & SWD_DAP_MATCH_VALUE) ? 4 : 0;
    }
    else {
        extra = 4;
    }
    p += 1 + extra;
    return p > limit ? limit : p;
}

static uint32_t dap_info(uint8_t id, uint8_t *info) {
    const char *s = NULL;

    switch (id) {
        case DAP_ID_VENDOR: s = "wanguyouy"; break;
        case DAP_ID_PRODUCT: s = "CW32 SWD DAP"; break;
        case DAP_ID_SER_NUM: s = pico_serial_str; break;
        case DAP_ID_DEVICE_VENDOR: s = "CWANG"; break;
        case DAP_ID_DEVICE_NAME: s = "CW32L010"; break;
        case DAP_ID_BOARD_VENDOR: s = "wanguyouy"; break;
        case DAP_ID_BOARD_NAME: s = "CW Writer"; break;
        case DAP_ID_PRODUCT_FW_VER: s = "1.0.0"; break;
        default: break;
    }
    if (s) {
        /* Len includes the terminating NUL, per the DAP_Info definition. */
        size_t n = strlen(s) + 1;

        if (n > DAP_PACKET - 2) {
            n = DAP_PACKET - 2;
        }
        memcpy(info, s, n);
        info[n - 1] = 0;
        return (uint32_t)n;
    }
    switch (id) {
        case DAP_ID_DAP_FW_VER: /* protocol version string, NUL included */
            memcpy(info, "2.0.0", 6);
            return 6;
        case DAP_ID_CAPABILITIES: /* SWD only */
            /* AtomicCommands (bit 4) is implemented here but its batched
             * response layout has never been validated against a DAPLink
             * host, and Keil switches to 0x7F batching as soon as it is
             * advertised.  Packet Count is 1, so batching buys nothing. */
            info[0] = 0x01;
            return 1;
        case DAP_ID_PACKET_COUNT:
            info[0] = 1;
            return 1;
        case DAP_ID_PACKET_SIZE: /* SHORT, little endian */
            info[0] = DAP_PACKET;
            info[1] = 0;
            return 2;
        default:
            return 0;
    }
}

/* U1/U2 run from VDD_TGT, so the level shifters are dead while that rail is
 * off.  On boards that do not wire VDD_TGT to the target the sense node
 * floats, so the external-supply check cannot be trusted here: present 3.3 V
 * for the SWD session and remember whether the rail is ours. */
static void dap_power_target(void) {
    if (cw_vdd_get() != CW_VDD_3V3) {
        cw_vdd_force(CW_VDD_3V3);
    }
    rail_owned = cw_vdd_get() != CW_VDD_OFF;
    printf("[dap] VDD_TGT rail %s\n", rail_owned ? "3.3V" : "OFF (force failed)");
}

#if defined(CWW_DAP_ONLY)
/* Stand-alone probe: grab the target and stop its core. */
static void dap_probe_target(void) {
    uint32_t id = 0;

    if (!dap_attach_under_reset(&id)) {
        printf("[dap] no idcode on the SWD pins\n");
    }
}
#endif

static uint32_t dap_connect(const uint8_t *request, uint8_t *response) {
    uint8_t port = request[0];

    if (port == 0) { /* DAP_PORT_AUTODETECT */
        port = 1;
    }
    if (port == 1) { /* DAP_PORT_SWD */
        rescue_count = 0;
        dap_power_target();
        dap_ensure_pads();
#if defined(CWW_DAP_ONLY)
        dap_probe_target();
#endif
        dap_dp_clear();
        debug_port = 1;
        printf("[dap] connect SWD, %lu Hz, target %d mV%s\n", (unsigned long)swd_dap_clock(),
               cw_vdd_target_mv(), rail_owned ? " (rail on)" : "");
    }
    else {
        port = 0; /* DAP_PORT_DISABLED */
    }
    *response = port;
    return ((uint32_t)1 << 16) | 1;
}

static uint32_t dap_disconnect(uint8_t *response) {
    debug_port = 0;
    swd_dap_release();
#if !defined(CWW_DAP_ONLY)
    cw_isp_pads_release_from_swd();
#endif
    if (rail_owned && cw_vdd_configured() == CW_VDD_OFF) {
        cw_vdd_set(CW_VDD_OFF);
    }
    rail_owned = false;
    printf("[dap] disconnect\n");
    *response = DAP_RSP_OK;
    return 1;
}

static uint32_t dap_transfer_configure(const uint8_t *request, uint8_t *response) {
    swd_dap_set_idle(request[0]);
    retry_count = (uint16_t)(request[1] | (request[2] << 8));
    match_retry_count = (uint16_t)(request[3] | (request[4] << 8));
    *response = DAP_RSP_OK;
    return ((uint32_t)5 << 16) | 1;
}

static uint32_t dap_transfer(const uint8_t *request, uint8_t *response) {
    const uint8_t *req_head = request;
    uint8_t *rsp_head = response;
    const uint8_t *limit = request + DAP_PACKET;
    uint32_t rsp_count = 0;
    uint32_t rsp_value = XFER_OK;
    uint32_t post_read = 0;
    uint32_t check_write = 0;
    uint32_t count;
    uint32_t total;
    uint32_t data = 0;

    response += 2;
    transfer_abort = false;

    request += 2; /* command, DAP index */
    total = *request++;

    for (count = total; count != 0; count--) {
        uint8_t rq = *request++;

        if (rq & SWD_DAP_RNW) {
            if (post_read) {
                /* Deliver the data of the previous AP read. */
                if ((rq & (SWD_DAP_APNDP | SWD_DAP_MATCH_VALUE)) == SWD_DAP_APNDP) {
                    rsp_value = swd_xfer_retry(rq, &data, retry_count);
                }
                else {
                    rsp_value = swd_xfer_retry(DP_RDBUFF | SWD_DAP_RNW, &data, retry_count);
                    post_read = 0;
                }
                if (rsp_value != XFER_OK) {
                    break;
                }
                if (!rsp_room(rsp_head, response, 4)) {
                    rsp_value = XFER_OK;
                    break;
                }
                put32(response, data);
                response += 4;
            }
            if (rq & SWD_DAP_MATCH_VALUE) {
                uint32_t match_value = get32(request);
                uint32_t mretry = match_retry_count;

                request += 4;
                if (rq & SWD_DAP_APNDP) {
                    rsp_value = swd_xfer_retry(rq, NULL, retry_count);
                    if (rsp_value != XFER_OK) {
                        break;
                    }
                }
                do {
                    rsp_value = swd_xfer_retry(rq, &data, retry_count);
                    if (rsp_value != XFER_OK) {
                        break;
                    }
                } while ((data & match_mask) != match_value && mretry-- && !transfer_abort);
                if (rsp_value != XFER_OK) {
                    break;
                }
                if ((data & match_mask) != match_value) {
                    rsp_value |= XFER_MISMATCH;
                }
                if (rsp_value != XFER_OK) {
                    break;
                }
            }
            else if (rq & SWD_DAP_APNDP) {
                if (!post_read) {
                    rsp_value = swd_xfer_retry(rq, NULL, retry_count);
                    if (rsp_value != XFER_OK) {
                        break;
                    }
                    post_read = 1;
                }
            }
            else {
                rsp_value = swd_xfer_retry(rq, &data, retry_count);
                if (rsp_value != XFER_OK) {
                    break;
                }
                if (!rsp_room(rsp_head, response, 4)) {
                    rsp_value = XFER_OK;
                    break;
                }
                put32(response, data);
                response += 4;
            }
            check_write = 0;
        }
        else {
            if (post_read) {
                rsp_value = swd_xfer_retry(DP_RDBUFF | SWD_DAP_RNW, &data, retry_count);
                if (rsp_value != XFER_OK) {
                    break;
                }
                if (!rsp_room(rsp_head, response, 4)) {
                    rsp_value = XFER_OK;
                    break;
                }
                put32(response, data);
                response += 4;
                post_read = 0;
            }
            data = get32(request);
            request += 4;
            if (rq & SWD_DAP_MATCH_MASK) {
                match_mask = data;
                rsp_value = XFER_OK;
            }
            else {
                rsp_value = swd_xfer_retry(rq, &data, retry_count);
                if (rsp_value != XFER_OK) {
                    break;
                }
                check_write = 1;
            }
        }
        rsp_count++;
        if (transfer_abort) {
            break;
        }
    }

    if (rsp_value == XFER_OK) {
        if (post_read) {
            rsp_value = swd_xfer_retry(DP_RDBUFF | SWD_DAP_RNW, &data, retry_count);
            if (rsp_value == XFER_OK && rsp_room(rsp_head, response, 4)) {
                put32(response, data);
                response += 4;
            }
        }
        else if (check_write) {
            (void)swd_xfer_retry(DP_RDBUFF | SWD_DAP_RNW, NULL, retry_count);
        }
    }

    {
        const uint8_t *end = req_head + 3; /* command, DAP index, count */

        for (count = 0; count < total; count++) {
            end = dap_item_end(end, limit);
        }
        rsp_head[0] = (uint8_t)rsp_count;
        rsp_head[1] = (uint8_t)rsp_value;
        return ((uint32_t)(end - req_head - 1) << 16) | (uint32_t)(response - rsp_head);
    }
}

static uint32_t dap_transfer_block(const uint8_t *request, uint8_t *response) {
    uint8_t *rsp_head = response;
    uint32_t total = (uint32_t)request[2] | ((uint32_t)request[3] << 8);
    uint32_t count = total;
    uint8_t rq = request[4];
    bool is_read = (rq & SWD_DAP_RNW) != 0;
    uint32_t rsp_count = 0;
    uint32_t rsp_value = XFER_OK;
    uint32_t data;

    request += 5;
    response += 3;
    transfer_abort = false;

    if (count != 0) {
        if (is_read) {
            if (rq & SWD_DAP_APNDP) {
                rsp_value = swd_xfer_retry(rq, NULL, retry_count);
            }
            while (rsp_value == XFER_OK && count--) {
                if (count == 0 && (rq & SWD_DAP_APNDP)) {
                    rq = DP_RDBUFF | SWD_DAP_RNW;
                }
                rsp_value = swd_xfer_retry(rq, &data, retry_count);
                if (rsp_value != XFER_OK) {
                    break;
                }
                if (!rsp_room(rsp_head, response, 4)) {
                    break;
                }
                put32(response, data);
                response += 4;
                rsp_count++;
            }
        }
        else {
            while (count--) {
                data = get32(request);
                request += 4;
                rsp_value = swd_xfer_retry(rq, &data, retry_count);
                if (rsp_value != XFER_OK) {
                    break;
                }
                rsp_count++;
                if (transfer_abort) {
                    break;
                }
            }
            if (rsp_value == XFER_OK) {
                (void)swd_xfer_retry(DP_RDBUFF | SWD_DAP_RNW, NULL, retry_count);
            }
        }
    }

    rsp_head[0] = (uint8_t)rsp_count;
    rsp_head[1] = (uint8_t)(rsp_count >> 8);
    rsp_head[2] = (uint8_t)rsp_value;

    return ((uint32_t)(4u + (is_read ? 0u : 4u * total)) << 16) | (uint32_t)(response - rsp_head);
}

static uint32_t dap_write_abort(const uint8_t *request, uint8_t *response) {
    uint32_t data = get32(request + 1);

    (void)swd_dap_transfer(DP_ABORT, &data);
    *response = DAP_RSP_OK;
    return ((uint32_t)5 << 16) | 1;
}

static uint32_t dap_delay(const uint8_t *request, uint8_t *response) {
    uint32_t us = (uint32_t)request[0] | ((uint32_t)request[1] << 8);

    if (us > 20000) {
        us = 20000;
    }
    swd_dap_delay_us(us);
    *response = DAP_RSP_OK;
    return ((uint32_t)2 << 16) | 1;
}

static uint32_t dap_reset_target(uint8_t *response) {
    uint32_t id = 0;

    dap_ensure_pads();
    swd_dap_take_nrst(true);
    swd_dap_nreset(0);
    vTaskDelay(pdMS_TO_TICKS(10));
    swd_dap_nreset(1);
    /* The DP comes back in JTAG mode after a real reset, but on this part a
     * select sent to a DP that is still in SWD mode pushes it back to JTAG.
     * Probe first and only select when the probe fails. */
    if (debug_port == 1) {
        swd_dap_line_reset();
        if (swd_dap_transfer(0x02, &id) != SWD_DAP_ACK_OK) {
            swd_dap_enter_swd();
            if (swd_dap_transfer(0x02, &id) != SWD_DAP_ACK_OK) {
                printf("[dap] reset done, no idcode after select\n");
            }
        }
dap_dp_clear();
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    response[0] = DAP_RSP_OK;
    response[1] = 0; /* no target-specific reset sequence */
    return ((uint32_t)1 << 16) | 2;
}

static uint32_t dap_swj_pins(const uint8_t *request, uint8_t *response) {
    uint8_t value = request[0];
    uint8_t select = request[1];
    uint32_t wait = (uint32_t)request[2] | ((uint32_t)request[3] << 8) |
                    ((uint32_t)request[4] << 16) | ((uint32_t)request[5] << 24);

    dap_ensure_pads();
    swd_dap_swj_pins(value, select);

    if (wait) {
        uint32_t deadline = (wait > 5000) ? 5000 : wait;

        while (deadline) {
            int pins = swd_dap_pins();

            if (((pins ^ value) & select) == 0) {
                break;
            }
            swd_dap_delay_us(10);
            deadline = (deadline > 10) ? deadline - 10 : 0;
        }
    }

    /* Response is [status][pin state]; the state belongs in byte 1, and a
     * tools that reads it as the status byte aborts the session. */
    response[0] = DAP_RSP_OK;
    response[1] = (uint8_t)swd_dap_pins();
    return ((uint32_t)6 << 16) | 2;
}

static uint32_t dap_swj_clock(const uint8_t *request, uint8_t *response) {
    uint32_t hz = get32(request);

    if (hz == 0) {
        *response = DAP_RSP_ERR;
    }
    else {
        swd_dap_set_clock(hz);
        *response = DAP_RSP_OK;
    }
    return ((uint32_t)4 << 16) | 1;
}

static uint32_t dap_swj_sequence(const uint8_t *request, uint8_t *response) {
    uint32_t count = request[0];
    uint32_t bytes;

    if (count == 0) {
        count = 256;
    }
    /* The select is not idempotent on this part: repeating it on a DP that is
     * still in SWD mode pushes it back to JTAG.  Treat the host's select as
     * "make sure the DP is in SWD": resync with a line reset, read IDCODE, and
     * only send the select if the probe failed. */
    if (debug_port == 1 && count == 16 && request[1] == 0x9E && request[2] == 0xE7) {
        uint32_t id = 0;
        bool ok = false;

        if (dap_ensure_pads()) {
            swd_dap_line_reset();
            /* The first request after a line reset can be missed, and a select
             * sent to a DP that is already in SWD pushes it back to JTAG, so
             * only give up after a real poll. */
            for (int i = 0; i < 8 && !ok; i++) {
                ok = swd_dap_transfer(0x02, &id) == SWD_DAP_ACK_OK;
            }
            if (!ok) {
                printf("[dap] host select, DP not in SWD, selecting\n");
                swd_dap_enter_swd();
                ok = swd_dap_transfer(0x02, &id) == SWD_DAP_ACK_OK;
                if (!ok) {
                    printf("[dap] host select, still no idcode\n");
                    *response = DAP_RSP_ERR;
                    return ((uint32_t)3 << 16) | 1;
                }
            }
            dap_dp_clear();
            *response = DAP_RSP_OK;
        }
        else {
            *response = DAP_RSP_ERR;
        }
        return ((uint32_t)3 << 16) | 1;
    }
    if (dap_ensure_pads()) {
        swd_dap_swj_sequence(count, request + 1);
/* A line reset (all ones) leaves STICKYORUN latched; the tool writes
         * CTRL/STAT right after it and would get FAULT. */
        if (debug_port == 1) {
            dap_dp_clear();
        }
        *response = DAP_RSP_OK;
    }
    else {
        *response = DAP_RSP_ERR;
    }
    bytes = (count + 7) / 8;
    return (((bytes + 1) << 16) | 1);
}

static uint32_t dap_swd_configure(const uint8_t *request, uint8_t *response) {
    swd_dap_configure(request[0]);
    *response = DAP_RSP_OK;
    return ((uint32_t)1 << 16) | 1;
}

static uint32_t dap_swd_sequence(const uint8_t *request, uint8_t *response) {
    uint32_t sequence_count = *request++;
    uint32_t req_len = 1;
    uint32_t rsp_len = 1;

    *response++ = DAP_RSP_OK;
    dap_ensure_pads();

    while (sequence_count--) {
        uint32_t info = *request++;
        uint32_t bits = info & SWD_DAP_SEQ_CLK;
        uint32_t bytes;

        if (bits == 0) {
            bits = 64;
        }
        bytes = (bits + 7) / 8;
        swd_dap_swd_sequence(info, request, response);
        if (info & SWD_DAP_SEQ_DIN) {
            req_len += 1;
            response += bytes;
            rsp_len += bytes;
        }
        else {
            request += bytes;
            req_len += 1 + bytes;
        }
    }
    swd_dap_swj_pins(1, 0x02); /* leave SWDIO driven high */

    return ((req_len << 16) | rsp_len);
}

static uint32_t dap_process(const uint8_t *request, uint8_t *response) {
    uint32_t n;

    if (request[0] == ID_DAP_EXECUTE_COMMANDS) {
        const uint8_t *rq = request + 2;
        uint32_t cnt = request[1];
        uint32_t len = 2;

        response[0] = ID_DAP_EXECUTE_COMMANDS;
        response[1] = (uint8_t)cnt;
        while (cnt--) {
            uint32_t m = dap_process(rq, response + len);

            len += m & 0xFFFF;
            rq += m >> 16;
        }
        return ((uint32_t)(rq - request) << 16) | len;
    }

    *response++ = request[0];

    switch (request[0]) {
        case ID_DAP_INFO: {
            uint32_t k = dap_info(request[1], response + 1);

            response[0] = (uint8_t)k;
            n = ((uint32_t)1 << 16) | (1 + k);
            break;
        }
        case ID_DAP_HOST_STATUS:
            *response = (request[1] == 0 || request[1] == 1) ? DAP_RSP_OK : DAP_RSP_ERR;
            n = ((uint32_t)1 << 16) | 1;
            break;
        case ID_DAP_CONNECT:
            n = dap_connect(request + 1, response);
            break;
        case ID_DAP_DISCONNECT:
            n = dap_disconnect(response);
            break;
        case ID_DAP_TRANSFER_CONFIGURE:
            n = dap_transfer_configure(request + 1, response);
            break;
        case ID_DAP_TRANSFER:
            n = dap_transfer(request, response);
            break;
        case ID_DAP_TRANSFER_BLOCK:
            n = dap_transfer_block(request, response);
            break;
        case ID_DAP_TRANSFER_ABORT:
            transfer_abort = true;
            *response = DAP_RSP_OK;
            n = ((uint32_t)1 << 16) | 1;
            break;
        case ID_DAP_WRITE_ABORT:
            n = dap_write_abort(request + 1, response);
            break;
        case ID_DAP_DELAY:
            n = dap_delay(request + 1, response);
            break;
        case ID_DAP_RESET_TARGET:
            n = dap_reset_target(response);
            break;
        case ID_DAP_SWJ_PINS:
            n = dap_swj_pins(request + 1, response);
            break;
        case ID_DAP_SWJ_CLOCK:
            n = dap_swj_clock(request + 1, response);
            break;
        case ID_DAP_SWJ_SEQUENCE:
            n = dap_swj_sequence(request + 1, response);
            break;
        case ID_DAP_SWD_CONFIGURE:
            n = dap_swd_configure(request + 1, response);
            break;
        case ID_DAP_SWD_SEQUENCE:
            n = dap_swd_sequence(request + 1, response);
            break;
        default:
            response[-1] = ID_DAP_INVALID;
            *response = DAP_RSP_ERR;
            n = ((uint32_t)1 << 16) | 1;
            break;
    }
    /* Handlers report parameters only: add the command byte to the request
     * length and the echoed command to the response length. */
    return n + (((uint32_t)1 << 16) | 1);
}

static void dap_send_pending(void) {
    if (!rsp_pending) {
        return;
    }
    if (tud_hid_report(0, rsp_buf, DAP_PACKET)) {
        rsp_pending = false;
    }
}

bool dap_hid_connected(void) {
    return debug_port == 1;
}

void tud_hid_set_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type,
                           uint8_t const *buffer, uint16_t bufsize) {
    uint32_t n;
    uint16_t len;

    (void)itf;
    (void)report_id;
    if (report_type != HID_REPORT_TYPE_OUTPUT) {
        return;
    }
    memset(req_buf, 0, sizeof(req_buf));
    memcpy(req_buf, buffer, bufsize < DAP_PACKET ? bufsize : DAP_PACKET);

    n = dap_process(req_buf, rsp_buf);
    len = (uint16_t)(n & 0xFFFF);
    /* Control commands only: a download is thousands of Transfer/TransferBlock
     * reports and one printf each would stall the session on the UART. */
    if (req_buf[0] != ID_DAP_TRANSFER && req_buf[0] != ID_DAP_TRANSFER_BLOCK) {
        printf("[dap] %02X %02X %02X %02X %02X -> %02X\n", req_buf[0], req_buf[1], req_buf[2],
               req_buf[3], req_buf[4], rsp_buf[1]);
    }
    if (len > DAP_PACKET) {
        len = DAP_PACKET;
    }
    memset(rsp_buf + len, 0, DAP_PACKET - len);

    rsp_pending = true;
    dap_send_pending();
}

uint16_t tud_hid_get_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type,
                               uint8_t *buffer, uint16_t reqlen) {
    (void)itf;
    (void)report_id;
    (void)report_type;
    memset(buffer, 0, reqlen);
    return reqlen;
}

void tud_hid_report_complete_cb(uint8_t itf, uint8_t const *report, uint16_t len) {
    (void)itf;
    (void)report;
    (void)len;
    dap_send_pending();
}

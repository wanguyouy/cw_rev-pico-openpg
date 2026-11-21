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
#define DP_ABORT  0x0Cu

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

/* Runs one SWD transfer, retrying WAIT responses like DAPLink does. */
static uint8_t swd_xfer_retry(uint8_t rq, uint32_t *data, uint32_t retry) {
    uint8_t ack;

    do {
        ack = swd_dap_transfer(rq, data);
    } while (ack == XFER_WAIT && retry-- && !transfer_abort);
    return ack;
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
        size_t n = strlen(s);

        if (n > DAP_PACKET - 2) {
            n = DAP_PACKET - 2;
        }
        memcpy(info, s, n);
        return (uint32_t)n;
    }
    switch (id) {
        case DAP_ID_DAP_FW_VER: /* "2.0.0" with terminator, as DAPLink sends */
            memcpy(info, "2.0.0", 6);
            return 6;
        case DAP_ID_CAPABILITIES: /* SWD + atomic commands */
            info[0] = 0x11;
            return 1;
        case DAP_ID_PACKET_COUNT:
            info[0] = 1;
            return 1;
        case DAP_ID_PACKET_SIZE:
            info[0] = DAP_PACKET;
            return 1;
        default:
            return 0;
    }
}

/* U1/U2 run from VDD_TGT, so the level shifters are dead while the target
 * has no supply.  Honour the tool's F5 setting; if it left the rail off and
 * nothing powers the target, enable 3.3 V for this session. */
static void dap_power_target(void) {
    cw_vdd_apply();
    if (cw_vdd_get() == CW_VDD_OFF && !cw_vdd_external()) {
        cw_vdd_set(CW_VDD_3V3);
        rail_owned = cw_vdd_get() != CW_VDD_OFF;
    }
}

static uint32_t dap_connect(const uint8_t *request, uint8_t *response) {
    uint8_t port = request[0];

    if (port == 0) { /* DAP_PORT_AUTODETECT */
        port = 1;
    }
    if (port == 1) { /* DAP_PORT_SWD */
        dap_power_target();
        dap_ensure_pads();
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
    cw_isp_pads_release_from_swd();
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
    dap_ensure_pads();
    swd_dap_nreset(0);
    vTaskDelay(pdMS_TO_TICKS(10));
    swd_dap_nreset(1);
    vTaskDelay(pdMS_TO_TICKS(10));
    response[0] = DAP_RSP_OK;
    response[1] = 0; /* no target-specific reset sequence */
    return 2;
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

    *response = (uint8_t)swd_dap_pins();
    return ((uint32_t)6 << 16) | 1;
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
    if (dap_ensure_pads()) {
        swd_dap_swj_sequence(count, request + 1);
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
            n = 1;
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

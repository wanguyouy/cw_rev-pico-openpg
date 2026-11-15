/*
 * Offline programming: flash the image stored in the "fw" partition into the
 * target with no PC attached.
 *
 * Frame layout follows a real CW_Programmer session (captured from USB):
 *   65 07 20 00 00 00 08 00 20       set RAM base 0x20000800
 *   65 93 27 00 00 <144 bytes>       download the flash helper
 *   65 f3 28 <off lo> <off hi> <240> write 240 bytes
 *   65 04 29 <off lo> <off hi> <len> read back
 *   65 07 40 00 00 00 00 00 00       jump to the application
 */

#include "cw_offline.h"

#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "cw_isp_prog.h"
#include "fw_store.h"
#include "ui_display.h"

#define CW_BOOT_GPIO 0
#define CW_CHUNK     240
#define BOOT_HOLD_MS 1000

static const char *TAG = "cwoffline";

/* Flash programming helper, 144 bytes, downloaded to RAM by CW_Programmer. */
static const uint8_t WRAM_HELPER[144] = {
    0xf0, 0xb5, 0x03, 0x00, 0x12, 0x4c, 0x01, 0x20, 0x20, 0x60, 0x20, 0x25,
    0x98, 0x07, 0x11, 0xd1, 0x08, 0xe0, 0x67, 0x6a, 0x2f, 0x40, 0xfc, 0xd1,
    0x1e, 0x68, 0x86, 0x42, 0x14, 0xd1, 0x09, 0x1d, 0x1b, 0x1d, 0x12, 0x1f,
    0x04, 0x2a, 0x05, 0xd3, 0x08, 0x68, 0x18, 0x60, 0xf1, 0xe7, 0x49, 0x1c,
    0x5b, 0x1c, 0x52, 0x1e, 0x00, 0x2a, 0x09, 0xd0, 0x08, 0x78, 0x18, 0x70,
    0x67, 0x6a, 0x2f, 0x40, 0xfc, 0xd1, 0x1e, 0x78, 0x86, 0x42, 0xf2, 0xd0,
    0x00, 0x20, 0xf0, 0xbd, 0x01, 0x20, 0xf0, 0xbd, 0x00, 0x20, 0x02, 0x40,
    0x38, 0xb5, 0x83, 0x18, 0x5b, 0x1e, 0xec, 0x24, 0xe4, 0x00, 0xa0, 0x42,
    0x02, 0xd3, 0x16, 0x34, 0xa3, 0x42, 0x09, 0xd3, 0x80, 0x24, 0x64, 0x03,
    0xa0, 0x42, 0x0d, 0xd3, 0xfb, 0x25, 0xed, 0x00, 0x2d, 0x68, 0x2c, 0x19,
    0xa3, 0x42, 0x07, 0xd2, 0xff, 0xf7, 0xc0, 0xff, 0x00, 0x28, 0x01, 0xd0,
    0x00, 0x20, 0x32, 0xbd, 0x98, 0x20, 0x32, 0xbd, 0x93, 0x20, 0x32, 0xbd,
};

static uint8_t tx_frame[264];
static uint8_t rx_frame[264];

static uint16_t crc16_x25(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 1) {
                crc = (uint16_t)((crc >> 1) ^ 0x8408);
            }
            else {
                crc >>= 1;
            }
        }
    }
    return (uint16_t)(crc ^ 0xFFFF);
}

/* Sends one ISP command and leaves the answer in rx_frame. */
static int isp_cmd(uint8_t cmd, const uint8_t *params, uint8_t plen) {
    uint16_t blen = (uint16_t)(1 + plen);
    tx_frame[0] = 0x65;
    tx_frame[1] = (uint8_t)blen;
    tx_frame[2] = cmd;
    if (plen > 0 && params != NULL) {
        memcpy(tx_frame + 3, params, plen);
    }
    uint16_t crc = crc16_x25(tx_frame, (size_t)(2 + blen));
    tx_frame[2 + blen] = (uint8_t)(crc & 0xFF);
    tx_frame[3 + blen] = (uint8_t)(crc >> 8);

    uint16_t rxlen = 0;
    int rc = cw_isp_link_xfer(tx_frame, (uint16_t)(blen + 4), rx_frame, sizeof(rx_frame), &rxlen);
    if (rc != 0) {
        return rc;
    }
    if (rxlen < 3 || rx_frame[2] != 0x00) {
        ESP_LOGE(TAG, "cmd %02X failed: rxlen %u status %02X", cmd, (unsigned)rxlen,
                 rxlen >= 3 ? rx_frame[2] : 0xFF);
        return -1;
    }
    return 0;
}

static void set_progress(uint32_t done, uint32_t total) {
    char text[22];
    snprintf(text, sizeof(text), "WR %lu%%", (unsigned long)(done * 100u / total));
    ui_set_status(text);
}

int cw_offline_run(void) {
    if (fw_store_init() != 0) {
        ui_set_status("NO FW PART");
        return -1;
    }
    if (!fw_store_valid()) {
        ui_set_status("NO FW");
        return -1;
    }

    uint32_t total = fw_store_size();
    static uint8_t chunk[CW_CHUNK];
    static uint8_t verify[CW_CHUNK];
    static uint8_t params[CW_CHUNK + 2];
    ui_set_file(fw_store_name());
    ui_set_status("OFFLINE");

    if (isp_cmd(0x10, NULL, 0) != 0) {
        ui_set_status("NO TARGET");
        return -2;
    }

    ui_set_status("ERASING");
    /* Match the vendor tool: 0x24 carries 8 x 0xFF. */
    static const uint8_t erase_params[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    if (isp_cmd(0x24, erase_params, sizeof(erase_params)) != 0) {
        ui_set_status("ERASE FAIL");
        return -3;
    }

    ui_set_status("HELPER");
    static const uint8_t ram_base[6] = {0x00, 0x00, 0x00, 0x08, 0x00, 0x20};
    if (isp_cmd(0x20, ram_base, sizeof(ram_base)) != 0) {
        ui_set_status("HELPER FAIL");
        return -4;
    }
    uint8_t helper[146] = {0x00, 0x00};
    memcpy(helper + 2, WRAM_HELPER, sizeof(WRAM_HELPER));
    if (isp_cmd(0x27, helper, sizeof(helper)) != 0) {
        ui_set_status("HELPER FAIL");
        return -4;
    }

    /* The helper download left the base address pointing at RAM (0x20000800);
     * the vendor tool always resets it to flash offset 0 before 0x28. */
    static const uint8_t flash_base[6] = {0};
    if (isp_cmd(0x20, flash_base, sizeof(flash_base)) != 0) {
        ui_set_status("WRITE FAIL");
        return -6;
    }

    for (uint32_t off = 0; off < total; off += CW_CHUNK) {
        uint16_t valid = (uint16_t)((total - off) > CW_CHUNK ? CW_CHUNK : (total - off));
        uint16_t count = CW_CHUNK;
        if (fw_store_read(off, chunk, valid) != 0) {
            ui_set_status("FLASH READ ERR");
            return -5;
        }
        if (valid < CW_CHUNK) {
            memset(chunk + valid, 0x00, CW_CHUNK - valid);
        }
        params[0] = (uint8_t)(off & 0xFF);
        params[1] = (uint8_t)(off >> 8);
        memcpy(params + 2, chunk, count);
        if (isp_cmd(0x28, params, (uint8_t)(count + 2)) != 0) {
            ui_set_status("WRITE FAIL");
            return -6;
        }
        set_progress(off + valid, total);
    }

    ui_set_status("VERIFY");
    if (isp_cmd(0x20, flash_base, sizeof(flash_base)) != 0) {
        ui_set_status("VERIFY FAIL");
        return -7;
    }
    for (uint32_t off = 0; off < total; off += CW_CHUNK) {
        uint16_t count = (uint16_t)((total - off) > CW_CHUNK ? CW_CHUNK : (total - off));
        uint8_t params[3] = {(uint8_t)(off & 0xFF), (uint8_t)(off >> 8), (uint8_t)count};
        if (isp_cmd(0x29, params, sizeof(params)) != 0) {
            ui_set_status("VERIFY FAIL");
            return -7;
        }
        if (fw_store_read(off, verify, count) != 0) {
            ui_set_status("FLASH READ ERR");
            return -5;
        }
        if (memcmp(rx_frame + 3, verify, count) != 0) {
            ESP_LOGE(TAG, "mismatch at %u", (unsigned)off);
            ui_set_status("CRC MISMATCH");
            return -8;
        }
    }

    ui_set_status("RESET");
    static const uint8_t jump[6] = {0};
    isp_cmd(0x40, jump, sizeof(jump));

    char done[24];
    snprintf(done, sizeof(done), "DONE 0X%08X", (unsigned)fw_store_auth());
    ui_set_status(done);
    ESP_LOGI(TAG, "offline flashing finished: %u bytes", (unsigned)total);
    return 0;
}

void cw_offline_task(void) {
    static bool pressed;
    static bool running;
    static uint32_t start_ms;

    if (running) {
        return;
    }
    if (gpio_get_level(CW_BOOT_GPIO) != 0) {
        pressed = false;
        return;
    }

    uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    if (!pressed) {
        pressed = true;
        start_ms = now;
        return;
    }
    if ((now - start_ms) < BOOT_HOLD_MS) {
        return;
    }

    pressed = false;
    running = true;
    ESP_LOGI(TAG, "BOOT held: starting offline programming");
    cw_offline_run();
    running = false;
}

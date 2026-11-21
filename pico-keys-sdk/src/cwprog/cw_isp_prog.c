/*
 * CW32L010 ISP link for ESP32-S3 - transparent bridge.
 *
 * The CW32L010 bootloader is entered by holding NRST low while a 50 kHz
 * square wave is applied to UART1_RXD (PA07), then releasing NRST.  After
 * that the chip speaks the CW32 ISP frame protocol on UART1:
 *
 *   0x65 | len | body | CRC16/X25(lo, hi)      CRC over header+len+body
 *
 * CW_Programmer runs on the PC against the emulated "CW Writer" (see
 * ccid.c).  Every ISP frame it sends is handed to cw_isp_link_xfer(), which
 * pushes it out on UART1 and returns the chip's own answer, so the PC really
 * programs the target instead of a simulated flash array.
 *
 * Wiring (5 wires):
 *   ESP32 3V3        -> target VDDIN (3.3V)
 *   ESP32 GND        -> target GND
 *   ESP32 GPIO4 (TX) -> PA07 (UART1_RXD / SWDIO)
 *   ESP32 GPIO5 (RX) <- PA08 (UART1_TXD / SWCLK)
 *   ESP32 GPIO6      -> NRST
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "esp_rom_gpio.h"
#include "soc/gpio_sig_map.h"
#include "soc/gpio_struct.h"

#include "cw_isp_prog.h"
#include "cw_vdd.h"
#include "ui_display.h"

#define CW_PIN_TX   GPIO_NUM_4
#define CW_PIN_RX   GPIO_NUM_5
#define CW_PIN_NRST GPIO_NUM_6
#define CW_PIN_DIR1 GPIO_NUM_15 /* U1: GPIO4 <-> PA07 */
#define CW_PIN_DIR2 GPIO_NUM_16 /* U2: GPIO5 <-> PA08 */

#define CW_UART UART_NUM_1
#define CW_BAUD 115200

#define CW_RX_MAX 262 /* largest answer frame: ReadData 255 bytes + header/CRC */

static int cw_link_up;
static int cw_target_in_isp;
static uint32_t cw_written_bytes;

/* Mirror the running ISP command on the OLED.  Only the state is written
 * here: ui_task() does the (slow) I2C transfer on the main loop. */
static void ui_report_cmd(uint8_t cmd, uint16_t txlen) {
    char text[22];
    switch (cmd) {
    case 0x24:
        cw_written_bytes = 0;
        ui_set_status("ERASING");
        break;
    case 0x26:
        ui_set_status("SECTOR ERASE");
        break;
    case 0x28:
        if (txlen > 9)
            cw_written_bytes += (uint32_t)(txlen - 9);
        snprintf(text, sizeof(text), "WRITE %lu.%luK",
                 (unsigned long)(cw_written_bytes / 1024),
                 (unsigned long)((cw_written_bytes % 1024) * 10 / 1024));
        ui_set_status(text);
        break;
    case 0x2A:
        ui_set_status("VERIFYING");
        break;
    case 0x40:
        ui_set_status("RESET");
        break;
    default:
        break;
    }
}

static uint16_t crc16_x25(const uint8_t *d, size_t n) {
    uint16_t crc = 0xFFFF;
    while (n--) {
        crc ^= *d++;
        for (int i = 0; i < 8; i++)
            crc = (crc & 1) ? (uint16_t)((crc >> 1) ^ 0x8408) : (uint16_t)(crc >> 1);
    }
    return (uint16_t)(crc ^ 0xFFFF);
}

static void cw_frame_crc(uint8_t *frame, size_t body_len) {
    uint16_t crc = crc16_x25(frame, body_len + 2);
    frame[body_len + 2] = (uint8_t)(crc & 0xFF);
    frame[body_len + 3] = (uint8_t)(crc >> 8);
}

/* CRC over a frame whose header has to be rebuilt: 65 | len | body. */
static uint16_t crc16_x25_header(uint8_t len, const uint8_t *body, size_t n) {
    uint8_t tmp[24];
    if (n > sizeof(tmp) - 2)
        n = sizeof(tmp) - 2;
    tmp[0] = 0x65;
    tmp[1] = len;
    memcpy(tmp + 2, body, n);
    return crc16_x25(tmp, n + 2);
}

/* ---------- UART transport ---------- */

/* Response codes the bootloader may return in body[0].  Anything else (line
 * noise, a half-duplex echo of our own frame) is skipped by the parser. */
static int cw_valid_ack(uint8_t c) {
    return c == 0x00 || c == 0x80 || (c >= 0x90 && c <= 0x99);
}

/* Read one response.
 *
 * The ISP line is half duplex: the echo of our own transmission overlaps the
 * first bytes of the answer, so the "65 len" header of short replies is often
 * destroyed while body+CRC arrive intact.  Two passes are used:
 *   1. scan the stream for a complete frame with a valid header and CRC;
 *   2. if that fails and the expected body length is known, rebuild the header
 *      (65 + len) and check the CRC against the tail of the stream.
 */
static int cw_read_frame(uint8_t *body, uint16_t *blen, uint32_t timeout_ms,
                         int expect_len) {
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    uint8_t win[300];
    int n = 0;

    /* No idle shortcut here: the target first echoes the frame it received and
     * only answers once the operation finished (flash programming takes a few
     * milliseconds), so the gap before the answer must not end the read. */
    while (xTaskGetTickCount() < deadline) {
        if (uart_read_bytes(CW_UART, win + n, 1, pdMS_TO_TICKS(5)) <= 0)
            continue;
        n++;
        for (int i = 0; i + 5 <= n; i++) {
            if (win[i] != 0x65)
                continue;
            int len = win[i + 1];
            if (len == 0 || i + 4 + len > n)
                continue;
            uint16_t crc = crc16_x25(win + i, (size_t)len + 2);
            if ((uint8_t)(crc & 0xFF) != win[i + len + 2] ||
                (uint8_t)(crc >> 8) != win[i + len + 3])
                continue;
            if (!cw_valid_ack(win[i + 2]))
                continue;
            memcpy(body, win + i + 2, len);
            *blen = len;
            return 0;
        }
        if (n > 260) {
            memmove(win, win + n - 260, 260);
            n = 260;
        }
    }

    if (expect_len > 0 && n >= expect_len + 2) {
        uint8_t *b = win + n - expect_len - 2;
        uint16_t crc = crc16_x25_header((uint8_t)expect_len, b, (size_t)expect_len);
        if ((uint8_t)(crc & 0xFF) == b[expect_len] &&
            (uint8_t)(crc >> 8) == b[expect_len + 1] && cw_valid_ack(b[0])) {
            memcpy(body, b, (size_t)expect_len);
            *blen = (uint16_t)expect_len;
            return 0;
        }
    }

    if (n > 0) {
        printf("[cwisp] rx (%d):", n);
        for (int i = 0; i < n && i < 32; i++)
            printf(" %02X", win[i]);
        printf("\n");
    }
    return -1;
}

/* ---------- Target entry ---------- */

static void cw_hw_init(void) {
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << CW_PIN_NRST,
        .mode = GPIO_MODE_INPUT_OUTPUT_OD, /* NRST has an external pull-up */
        /* 内部上拉必须关掉：目标断电时它会经目标 NRST 的 ESD 二极管把
         * VDD_TGT 充到 2 V 左右，cw_vdd_external() 因此误判目标自供电，
         * 拒绝打开电源轨，目标一直处在欠压状态（ISP/SWD 都不应答）。 */
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    gpio_set_level(CW_PIN_NRST, 1);

    /* 74LVC1T45 方向：ISP 时 PA07 线为 TX(高)、PA08 线为 RX(低)。
     * 这两脚悬空会让 SWCLK 方向不确定，导致 SWD 完全收不到时钟。 */
    gpio_config_t dir = {
        .pin_bit_mask = (1ULL << CW_PIN_DIR1) | (1ULL << CW_PIN_DIR2),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&dir);
    gpio_set_level(CW_PIN_DIR1, 1);
    gpio_set_level(CW_PIN_DIR2, 0);

    ledc_timer_config_t tcfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_1_BIT,
        .timer_num = LEDC_TIMER_1,
        .freq_hz = 50000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&tcfg);

    uart_config_t ucfg = {
        .baud_rate = CW_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(CW_UART, 2048, 2048, 0, NULL, 0);
    uart_param_config(CW_UART, &ucfg);
    uart_set_pin(CW_UART, CW_PIN_TX, CW_PIN_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

/* Drive PA07 with the LEDC square wave.
 *
 * GPIO4 is reserved by the UART driver (uart_set_pin() calls
 * esp_gpio_reserve() and IDF 5.5 has no way to release it), so the LEDC
 * driver logs "GPIO 4 is not usable" - it is only a warning, the signal is
 * still connected to the pad.  ledc_channel_config() must be given a real
 * gpio_num though: -1 trips an ARG_CHECK and the channel is never set up. */
static void cw_clock_on(void) {
    ledc_channel_config_t on = {
        .gpio_num = CW_PIN_TX,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_1,
        .duty = 1,
        .hpoint = 0,
    };
    ledc_channel_config(&on);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    esp_rom_gpio_connect_out_signal(CW_PIN_TX, LEDC_LS_SIG_OUT0_IDX + LEDC_CHANNEL_1,
                                    false, false);
}

/* 把 PA07/PA08 两个焊盘交回 UART1（TX/RX）。 */
static void cw_uart_restore(void) {
    uart_set_pin(CW_UART, CW_PIN_TX, CW_PIN_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    /* uart_set_pin() 只置 FUN_IE，不会关掉 RX 脚之前的输出使能；SWD 把
     * GPIO5 当 SWCLK 推挽输出过，这里必须手工放开，否则目标 TXD 被拉死。 */
    GPIO.enable_w1ts = 1u << CW_PIN_TX;
    GPIO.enable_w1tc = 1u << CW_PIN_RX;
    /* 复位两路电平转换方向（SWD 可能把它们留在别的状态） */
    gpio_set_level(CW_PIN_DIR1, 1);
    gpio_set_level(CW_PIN_DIR2, 0);
}

static void cw_clock_off(void) {
    ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 1); /* idle high */
    cw_uart_restore();
}

/* ---------- SWD pad hand-off ---------- */

static bool cw_pads_held_by_swd;

/* CMSIS-DAP took GPIO4/5/6 over: stop the ISP clock and keep the ISP link
 * off these pads until the debugger disconnects. */
void cw_isp_pads_hold_for_swd(void) {
    if (cw_pads_held_by_swd) {
        return;
    }
#if !defined(CWW_DAP_ONLY)
    ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 1);
#endif
    cw_pads_held_by_swd = true;
    cw_target_in_isp = 0;
    printf("[cwisp] pads handed to SWD\n");
}

void cw_isp_pads_release_from_swd(void) {
    if (!cw_pads_held_by_swd) {
        return;
    }
    cw_pads_held_by_swd = false;
    cw_uart_restore();
    cw_target_in_isp = 0;
    printf("[cwisp] pads back on UART1\n");
}

bool cw_isp_pads_held(void) {
    return cw_pads_held_by_swd;
}

/* Keep the chip in reset, feed 50 kHz into PA07, then release reset. */
static void cw_isp_entry(void) {
    /* 按工具的"供电/复位方式"准备目标 */
    cw_vdd_apply();
    if (cw_vdd_reset_mode() == CW_RST_POWER && cw_vdd_configured() != CW_VDD_OFF &&
        !cw_vdd_external()) {
        cw_vdd_set(CW_VDD_OFF);
        vTaskDelay(pdMS_TO_TICKS(50));
        cw_vdd_apply();
    }

    gpio_set_level(CW_PIN_NRST, 0);
    vTaskDelay(pdMS_TO_TICKS(5));

    cw_clock_on();
    vTaskDelay(pdMS_TO_TICKS(30));

    gpio_set_level(CW_PIN_NRST, 1);
    vTaskDelay(pdMS_TO_TICKS(20));

    cw_clock_off();
    uart_flush_input(CW_UART);
    vTaskDelay(pdMS_TO_TICKS(5));
}

/* ---------- Transparent ISP bridge ---------- */

static void cw_write(uint8_t *tx, uint16_t txlen) {
    uart_flush_input(CW_UART);
    uart_write_bytes(CW_UART, tx, txlen);
    uart_wait_tx_done(CW_UART, pdMS_TO_TICKS(200));
    vTaskDelay(pdMS_TO_TICKS(3));
}

/* Put the bootloader back in command mode and confirm it with a Query. */
static int cw_enter(void) {
    uint8_t q[5] = {0x65, 0x01, 0x10, 0x00, 0x00};
    cw_frame_crc(q, 1);

    for (int i = 0; i < 3; i++) {
        cw_isp_entry();
        vTaskDelay(pdMS_TO_TICKS(20));
        cw_write(q, sizeof(q));

        uint8_t body[64];
        uint16_t n = 0;
        if (cw_read_frame(body, &n, 500, 0) == 0 && body[0] == 0x00 && n > 5) {
            char name[24];
            size_t len = (size_t)n - 5;
            if (len >= sizeof(name))
                len = sizeof(name) - 1;
            memcpy(name, body + 5, len);
            name[len] = '\0';
            printf("[cwisp] target in ISP: %s\n", name);
            ui_set_chip(name);
            ui_set_status("ISP OK");
            cw_target_in_isp = 1;
            return 0;
        }
    }
    printf("[cwisp] no answer from target: check wiring (3V3/GND/PA07/PA08/NRST)\n");
    ui_set_status("NO TARGET");
    return -1;
}

/* 只做一次普通复位（不灌 50kHz），然后直接发 ISP 查询。
 * 用来判断"复位后芯片是不是还停在 BootLoader 里"——若仍应答，
 * 说明空片会一直留在 ISP，SWD 自然拿不到。 */
int cw_isp_query_cold(uint8_t *rx, uint16_t rxcap, uint16_t *rxlen) {
    uint8_t body[64];
    uint16_t n = 0;
    uint8_t q[5] = {0x65, 0x01, 0x10, 0x00, 0x00};

    if (cw_pads_held_by_swd) {
        return -1;
    }
    if (!cw_link_up) {
        cw_hw_init();
        cw_link_up = 1;
    }
    cw_frame_crc(q, 1);

    gpio_set_level(CW_PIN_NRST, 0);
    vTaskDelay(pdMS_TO_TICKS(5));
    gpio_set_level(CW_PIN_NRST, 1);
    vTaskDelay(pdMS_TO_TICKS(20));

    cw_uart_restore();
    cw_write(q, sizeof(q));
    if (cw_read_frame(body, &n, 600, 0) != 0) {
        cw_target_in_isp = 0;
        return -1;
    }
    if ((uint16_t)(n + 4u) > rxcap) {
        return -1;
    }
    rx[0] = 0x65;
    rx[1] = (uint8_t)n;
    memcpy(rx + 2, body, n);
    cw_frame_crc(rx, n);
    *rxlen = (uint16_t)(n + 4u);
    return 0;
}

int cw_isp_link_xfer(const uint8_t *tx, uint16_t txlen, uint8_t *rx, uint16_t rxcap,
                     uint16_t *rxlen) {
    uint8_t body[272];
    uint16_t n = 0;
    uint8_t cmd = (txlen > 2) ? tx[2] : 0;
    uint8_t blen = (txlen > 1) ? tx[1] : 0;
    int expect;
    uint32_t timeout;

    if (cw_pads_held_by_swd) {
        printf("[cwisp] SWD owns PA07/PA08, ISP refused\n");
        return -1;
    }

    switch (cmd) {
    case 0x10: expect = 0; timeout = 800; break;
    case 0x29:
        expect = (blen >= 4) ? (int)tx[5] + 1 : 0;
        timeout = 2000;
        break;
    case 0x2A: expect = 3; timeout = 6000; break;
    case 0x24: expect = 1; timeout = 6000; break;
    case 0x26: expect = 1; timeout = 3000; break;
    case 0x27: expect = 1; timeout = 2000; break;
    case 0x28: expect = 1; timeout = 2000; break;
    case 0x30: expect = 2; timeout = 2000; break;
    default:   expect = 1; timeout = 1500; break;
    }

    if (!cw_link_up) {
        cw_hw_init();
        cw_link_up = 1;
    }

    for (int attempt = 0; attempt < 2; attempt++) {
        if (!cw_target_in_isp && cw_enter() != 0)
            return -1;

        ui_report_cmd(cmd, txlen);
        cw_write((uint8_t *)tx, txlen);
        n = 0;
        if (cw_read_frame(body, &n, timeout, expect) == 0) {
            if ((uint16_t)(n + 4u) > rxcap)
                return -1;
            rx[0] = 0x65;
            rx[1] = (uint8_t)n;
            memcpy(rx + 2, body, n);
            cw_frame_crc(rx, n);
            *rxlen = (uint16_t)(n + 4u);

            if (cmd == 0x24)
                ui_set_status("ERASED");
            else if (cmd == 0x2A)
                ui_set_status("VERIFY DONE");
            else if (cmd == 0x40) { /* Jump: let the new firmware start from a reset */
                cw_target_in_isp = 0;
                ui_set_status("RUNNING");
                gpio_set_level(CW_PIN_NRST, 0);
                vTaskDelay(pdMS_TO_TICKS(10));
                gpio_set_level(CW_PIN_NRST, 1);
            }
            return 0;
        }
        cw_target_in_isp = 0; /* link lost: re-enter the bootloader and retry */
    }

    printf("[cwisp] cmd %02X: target did not answer\n", cmd);
    ui_set_status("ERROR");
    return -1;
}

void cw_isp_prog_start(void) {
    cw_hw_init();
    cw_link_up = 1;
}

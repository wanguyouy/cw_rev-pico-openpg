/*
 * Bit-banged SWD master (bring-up).
 *
 * GPIO4/5 are reserved by the UART1 driver (uart_set_pin() calls
 * esp_gpio_reserve() and IDF 5.5 has no release API), and GPIO4 is also
 * routed from LEDC while the ISP clock runs.  The GPIO driver therefore
 * refuses to touch these pads, so the bit-bang layer drives them straight
 * through the GPIO registers after re-muxing the pads to plain GPIO.
 *
 * PA07/PA08 are the CW32L010 SWD pads, but which one is SWDIO is not
 * documented clearly, so the mapping can be swapped at runtime.
 */

#include "swd.h"

#include <stdio.h>

#include "esp_rom_gpio.h"
#include "esp_rom_sys.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/gpio_sig_map.h"
#include "soc/gpio_struct.h"
#include "soc/io_mux_reg.h"

#define SWD_PIN_A 4 /* PA07 */
#define SWD_PIN_B 5 /* PA08 */
#define SWD_NRST  6
#define SWD_DIR_A 15 /* 74LVC1T45 U1 DIR: PA07 线 (A=GPIO4) */
#define SWD_DIR_B 16 /* 74LVC1T45 U2 DIR: PA08 线 (A=GPIO5) */

#define SWD_HALF_US 3

static int io_pin = SWD_PIN_A;
static int clk_pin = SWD_PIN_B;
static int io_dir = SWD_DIR_A;
static int clk_dir = SWD_DIR_B;
static int half_us = SWD_HALF_US;
static bool ready;
static int last_ack = -1;

#define SWD_ACK_OK    1
#define SWD_ACK_WAIT  2
#define SWD_ACK_FAULT 4

int swd_last_ack(void) {
    return last_ack;
}

static inline void swd_delay(void) {
    esp_rom_delay_us(half_us);
}

static inline void pin_write(int pin, int level) {
    if (level) {
        GPIO.out_w1ts = 1u << pin;
    }
    else {
        GPIO.out_w1tc = 1u << pin;
    }
}

static inline void pin_output_enable(int pin, bool en) {
    if (en) {
        GPIO.enable_w1ts = 1u << pin;
    }
    else {
        GPIO.enable_w1tc = 1u << pin;
    }
}

static inline int pin_read(int pin) {
    return (int)((GPIO.in >> pin) & 1u);
}

static volatile uint32_t *mux_reg(int pin) {
    return pin == SWD_PIN_A ? (volatile uint32_t *)IO_MUX_GPIO4_REG
                            : (volatile uint32_t *)IO_MUX_GPIO5_REG;
}

/* Route the pad to plain GPIO, make it an output, push-pull. */
static void pin_claim_out(int pin) {
    esp_rom_gpio_pad_select_gpio(pin);
    esp_rom_gpio_connect_out_signal(pin, SIG_GPIO_OUT_IDX, false, false);
    pin_output_enable(pin, true);
}

/* Route the pad to plain GPIO, release it and enable input + pull-up.
 * UART leaves FUN_IE/FUN_WPU in a state that makes GPIO.in read 0. */
static void pin_claim_in(int pin) {
    esp_rom_gpio_pad_select_gpio(pin);
    esp_rom_gpio_connect_out_signal(pin, SIG_GPIO_OUT_IDX, false, false);
    pin_output_enable(pin, false);
    *mux_reg(pin) |= (1u << 8) | (1u << 9); /* FUN_WPU | FUN_IE */
    *mux_reg(pin) &= ~(1u << 7);            /* FUN_WPD off */
}

static void swd_clk_pulse(void) {
    pin_write(clk_pin, 1);
    swd_delay();
    pin_write(clk_pin, 0);
    swd_delay();
}

static void swd_io_dir(bool out) {
    pin_write(io_dir, out ? 1 : 0);
    pin_output_enable(io_pin, out);
}

static void swd_write_bits(uint32_t v, int n) {
    for (int i = 0; i < n; i++) {
        pin_write(io_pin, (int)((v >> i) & 1u));
        swd_clk_pulse();
    }
}

static uint32_t swd_read_bits(int n) {
    uint32_t v = 0;

    for (int i = 0; i < n; i++) {
        pin_write(clk_pin, 1);
        swd_delay();
        if (pin_read(io_pin)) {
            v |= 1u << i;
        }
        pin_write(clk_pin, 0);
        swd_delay();
    }
    return v;
}

static int parity32(uint32_t v) {
    v ^= v >> 16;
    v ^= v >> 8;
    v ^= v >> 4;
    v &= 0x0F;
    return (0x6996 >> v) & 1;
}

static uint8_t swd_make_req(int apndp, int rnw, uint32_t addr) {
    uint8_t req = 0x81; /* start + park */

    if (apndp) {
        req |= 0x02;
    }
    if (rnw) {
        req |= 0x04;
    }
    if (addr & 0x4) {
        req |= 0x08;
    }
    if (addr & 0x8) {
        req |= 0x10;
    }
    if (__builtin_parity(req & 0x1E)) {
        req |= 0x20;
    }
    return req;
}

static void swd_line_reset(void) {
    swd_io_dir(true);
    pin_write(io_pin, 1);
    for (int i = 0; i < 56; i++) {
        swd_clk_pulse();
    }
}

static void swd_jtag_to_swd(void) {
    swd_io_dir(true);
    swd_write_bits(0xE79E, 16);
}

/* One DP/AP transaction.  Returns the raw ACK (1 = OK). */
static int swd_xfer(uint8_t req, uint32_t *data) {
    swd_io_dir(true);
    swd_write_bits(req, 8);

    /* 请求最后一位之后是一个完整时钟的转向周期（TRN），期间双方都不驱动，
     * 目标从下一个上升沿开始给 3 位 ACK。 */
    swd_io_dir(false);
    swd_clk_pulse();

    uint32_t ack = swd_read_bits(3);
    last_ack = (int)ack;
    if (ack == SWD_ACK_OK) {
        if (req & 0x04) {
            uint32_t v = swd_read_bits(32);
            (void)swd_read_bits(1); /* parity */
            swd_clk_pulse();        /* turnaround back to host */
            swd_io_dir(true);
            if (data) {
                *data = v;
            }
        }
        else {
            swd_io_dir(true);
            uint32_t v = data ? *data : 0;
            swd_write_bits(v, 32);
            swd_write_bits((uint32_t)parity32(v), 1);
        }
        pin_write(io_pin, 1);
        swd_clk_pulse();
        swd_clk_pulse();
        return SWD_ACK_OK;
    }

    /* WAIT / FAULT / protocol error: release the line and idle */
    swd_io_dir(true);
    pin_write(io_pin, 1);
    swd_clk_pulse();
    swd_clk_pulse();
    return (int)ack;
}

bool swd_init(int swap_pins) {
    /* Stop the ISP clock: its LEDC channel keeps driving the GPIO4 pad. */
    ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 1);

    io_pin = swap_pins ? SWD_PIN_B : SWD_PIN_A;
    clk_pin = swap_pins ? SWD_PIN_A : SWD_PIN_B;
    io_dir = swap_pins ? SWD_DIR_B : SWD_DIR_A;
    clk_dir = swap_pins ? SWD_DIR_A : SWD_DIR_B;

    pin_claim_in(io_pin);
    pin_claim_out(clk_pin);
    pin_claim_out(SWD_NRST);
    pin_claim_out(io_dir);
    pin_claim_out(clk_dir);
    pin_write(clk_pin, 0);
    pin_write(SWD_NRST, 1);
    pin_write(clk_dir, 1); /* SWCLK 始终 ESP32 -> 目标 */
    pin_write(io_dir, 1);

    ready = true;
    printf("[cwswd] link up (%d kHz), SWDIO=GPIO%d SWCLK=GPIO%d\n",
           1000 / (2 * SWD_HALF_US), io_pin, clk_pin);
    return true;
}

/* 一次完整的 line reset + JTAG-to-SWD + 读 IDCODE */
static int swd_read_id_once(uint32_t *out) {
    swd_line_reset();
    swd_jtag_to_swd();
    swd_line_reset();
    pin_write(io_pin, 1);
    swd_clk_pulse();
    swd_clk_pulse();
    return swd_xfer(swd_make_req(0, 1, 0x00), out);
}

/* 复位保持期间完成整次 IDCODE 读取，读完再放开复位（connect-under-reset）。
 * 目标固件若把 PA07/PA08 改成普通 GPIO，只有这样才能碰得到调试口。 */
static int swd_read_id_after_reset(uint32_t *out) {
    pin_write(SWD_NRST, 0);
    vTaskDelay(pdMS_TO_TICKS(5));
    swd_line_reset();
    swd_jtag_to_swd();
    swd_line_reset();
    pin_write(io_pin, 1);
    swd_clk_pulse();
    swd_clk_pulse();
    int rc = swd_xfer(swd_make_req(0, 1, 0x00), out);
    pin_write(SWD_NRST, 1);
    vTaskDelay(pdMS_TO_TICKS(2));
    return rc;
}

/* 复位 -> 放开 -> 等目标启动 -> 再连。
 * BootLoader 用 ISP 指令跳转到用户程序时不会复位外设，PA07/PA08 会停留在
 * UART1 复用态；只有真正走一次 NRST 才能恢复成 SWDIO/SWCLK。 */
static int swd_read_id_reset_first(uint32_t *out) {
    pin_write(SWD_NRST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    pin_write(SWD_NRST, 1);
    vTaskDelay(pdMS_TO_TICKS(30));
    return swd_read_id_once(out);
}

uint32_t swd_idcode(void) {
    uint32_t id = 0;

    if (!ready && !swd_init(0)) {
        return 0;
    }

    /* 慢速目标 / 长线时逐档降速重试 */
    static const int steps[] = {3, 6, 12, 25};

    for (size_t i = 0; i < sizeof(steps) / sizeof(steps[0]); i++) {
        half_us = steps[i];

        if (swd_read_id_once(&id) == SWD_ACK_OK) {
            printf("[cwswd] idcode ok at %d kHz\n", 1000 / (2 * half_us));
            return id;
        }
        if (swd_read_id_reset_first(&id) == SWD_ACK_OK) {
            printf("[cwswd] idcode ok at %d kHz (after reset)\n", 1000 / (2 * half_us));
            return id;
        }
        if (swd_read_id_after_reset(&id) == SWD_ACK_OK) {
            printf("[cwswd] idcode ok at %d kHz (under reset)\n", 1000 / (2 * half_us));
            return id;
        }
        id = 0;
    }
    half_us = SWD_HALF_US;
    return 0;
}

/* 空闲时 SWDIO 的实际电平（-1 = 未初始化） */
int swd_idle_level(void) {
    if (!ready) {
        return -1;
    }
    pin_output_enable(io_pin, false);
    swd_delay();
    return pin_read(io_pin);
}

/* 焊盘自检：低字节 SWDIO 读回、高字节 SWCLK 读回，正常 0x1010 */
int swd_pad_check(void) {
    int hi, lo, chi, clo;

    if (!ready) {
        return -1;
    }
    swd_io_dir(true);
    pin_write(io_pin, 1);
    swd_delay();
    swd_delay();
    hi = pin_read(io_pin);
    pin_write(io_pin, 0);
    swd_delay();
    swd_delay();
    lo = pin_read(io_pin);
    pin_output_enable(io_pin, false);

    pin_write(clk_pin, 1);
    swd_delay();
    swd_delay();
    chi = pin_read(clk_pin);
    pin_write(clk_pin, 0);
    swd_delay();
    swd_delay();
    clo = pin_read(clk_pin);

    return ((hi << 4) | lo) | (((chi << 4) | clo) << 8);
}

int swd_probe(void) {
    uint32_t id = swd_idcode();

    if (id == 0) {
        printf("[cwswd] no answer (check 3V3/GND/PA07/PA08/NRST)\n");
        return -1;
    }
    printf("[cwswd] DP IDCODE 0x%08X%s\n", (unsigned)id,
           (id & 0xFFF00000u) == 0x0BC00000u ? " (Cortex-M0+)" : "");
    return 0;
}

/* 不驱动、不复位，只读两脚：先都下拉读，再都上拉读。
 * 返回值 bit3..0 = (PA07 下拉, PA08 下拉, PA07 上拉, PA08 上拉)。
 * 判断方法：下拉读 1 -> 被目标推高；两种都是 0 -> 被拉低；下拉 0 / 上拉 1 -> 浮空。 */
int swd_pins_sample(void) {
    int pd_io, pd_clk, pu_io, pu_clk;

    ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 1);
    pin_claim_in(SWD_PIN_A);
    pin_claim_in(SWD_PIN_B);

    *mux_reg(SWD_PIN_A) &= ~(1u << 8);
    *mux_reg(SWD_PIN_B) &= ~(1u << 8);
    *mux_reg(SWD_PIN_A) |= (1u << 7);
    *mux_reg(SWD_PIN_B) |= (1u << 7);
    vTaskDelay(pdMS_TO_TICKS(2));
    pd_io = pin_read(SWD_PIN_A);
    pd_clk = pin_read(SWD_PIN_B);

    *mux_reg(SWD_PIN_A) &= ~(1u << 7);
    *mux_reg(SWD_PIN_B) &= ~(1u << 7);
    *mux_reg(SWD_PIN_A) |= (1u << 8);
    *mux_reg(SWD_PIN_B) |= (1u << 8);
    vTaskDelay(pdMS_TO_TICKS(2));
    pu_io = pin_read(SWD_PIN_A);
    pu_clk = pin_read(SWD_PIN_B);

    ready = false;
    return (pd_io << 3) | (pd_clk << 2) | (pu_io << 1) | pu_clk;
}

/* 两种引脚映射各试一遍，返回 "A:id/ack B:id/ack" 供 CDC 打印 */
void swd_probe_map(uint32_t *id_a, int *ack_a, uint32_t *id_b, int *ack_b) {
    for (int swap = 0; swap < 2; swap++) {
        ready = false;
        swd_init(swap);
        uint32_t id = swd_idcode();
        if (swap == 0) {
            *id_a = id;
            *ack_a = last_ack;
        }
        else {
            *id_b = id;
            *ack_b = last_ack;
        }
    }
    ready = false;
    swd_init(0);
}

/* 复位后目标引脚状态诊断。
 *
 * PA07 用上拉、PA08 用下拉，只读不驱动：芯片若还停在 ISP 引导程序里，
 * PA08(USART1_TXD) 会被目标推挽驱动成高电平，拉不下来；正常情况下
 * PA08 复位后是浮空输入，会被下拉拉低。
 *
 * bit1 = PA07 电平, bit0 = PA08 电平。 */
int swd_pins_after_reset(void) {
    ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 1);

    pin_claim_in(SWD_PIN_A);
    pin_claim_in(SWD_PIN_B);
    *mux_reg(SWD_PIN_A) |= (1u << 8);  /* PA07 上拉 */
    *mux_reg(SWD_PIN_B) &= ~(1u << 8); /* PA08 不要上拉 */
    *mux_reg(SWD_PIN_B) |= (1u << 7);  /* PA08 下拉 */

    pin_claim_out(SWD_NRST);
    pin_write(SWD_NRST, 0);
    vTaskDelay(pdMS_TO_TICKS(5));
    pin_write(SWD_NRST, 1);
    vTaskDelay(pdMS_TO_TICKS(20));

    int io = pin_read(SWD_PIN_A);
    int clk = pin_read(SWD_PIN_B);

    *mux_reg(SWD_PIN_B) &= ~(1u << 7);
    ready = false;
    return (io << 1) | clk;
}

/* 裸位流：复位 -> 线复位 -> JTAG2SWD -> 线复位 -> 发 0xA5 -> 连采 nbits。
 * 全 1 = 目标根本没驱动 SWDIO；出现 0 = 目标在应答，只是帧对齐不对。 */
uint64_t swd_raw_read(int nbits) {
    uint64_t v = 0;

    if (nbits > 40) {
        nbits = 40;
    }
    if (!ready && !swd_init(0)) {
        return 0;
    }

    pin_write(SWD_NRST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    pin_write(SWD_NRST, 1);
    vTaskDelay(pdMS_TO_TICKS(30));

    swd_line_reset();
    swd_jtag_to_swd();
    swd_line_reset();
    pin_write(io_pin, 1);
    swd_clk_pulse();
    swd_clk_pulse();

    swd_io_dir(true);
    swd_write_bits(swd_make_req(0, 1, 0x00), 8);
    swd_io_dir(false);

    for (int i = 0; i < nbits; i++) {
        pin_write(clk_pin, 1);
        swd_delay();
        if (pin_read(io_pin)) {
            v |= (uint64_t)1 << i;
        }
        pin_write(clk_pin, 0);
        swd_delay();
    }
    return v;
}

/* 自环：GPIO4/GPIO5 单独驱动、另一脚读回。两条线都要先从目标上摘下来。 */
int swd_loopback(void) {
    int r = 0;

    /* 两脚都切成 GPIO，先都当输入 */
    pin_claim_in(SWD_PIN_A);
    pin_claim_in(SWD_PIN_B);
    pin_write(SWD_PIN_A, 0);
    pin_write(SWD_PIN_B, 0);
    vTaskDelay(pdMS_TO_TICKS(2));

    /* A 驱动，B 读 */
    pin_output_enable(SWD_PIN_A, true);
    pin_write(SWD_PIN_A, 1);
    vTaskDelay(pdMS_TO_TICKS(2));
    if (pin_read(SWD_PIN_B)) r |= 1 << 3;
    pin_write(SWD_PIN_A, 0);
    vTaskDelay(pdMS_TO_TICKS(2));
    if (!pin_read(SWD_PIN_B)) r |= 1 << 2;
    pin_output_enable(SWD_PIN_A, false);

    /* B 驱动，A 读 */
    pin_output_enable(SWD_PIN_B, true);
    pin_write(SWD_PIN_B, 1);
    vTaskDelay(pdMS_TO_TICKS(2));
    if (pin_read(SWD_PIN_A)) r |= 1 << 1;
    pin_write(SWD_PIN_B, 0);
    vTaskDelay(pdMS_TO_TICKS(2));
    if (!pin_read(SWD_PIN_A)) r |= 1 << 0;
    pin_output_enable(SWD_PIN_B, false);

    ready = false;
    return r;
}

/*
 * CMSIS-DAP SWD engine on the CW Writer pads.
 *
 * The wire timing follows ARM's SW_DP.c exactly: every clock starts with a
 * falling edge, writes change SWDIO while the clock is high, reads sample
 * SWDIO while the clock is low, and the data word is LSB first with a
 * trailing even-parity bit.
 */

#include "swd_dap.h"

#include "driver/ledc.h"
#include "esp_cpu.h"
#include "esp_rom_gpio.h"
#include "freertos/FreeRTOS.h"
#include "sdkconfig.h"
#include "soc/gpio_sig_map.h"
#include "soc/gpio_struct.h"
#include "soc/io_mux_reg.h"

#define IO_PIN   4  /* PA07 */
#define CLK_PIN  5  /* PA08 */
#define NRST_PIN 6
#define DIR_IO   15 /* U1: GPIO4 <-> PA07 */
#define DIR_CLK  16 /* U2: GPIO5 <-> PA08 */

/* Power management is off in this firmware (CONFIG_PM_ENABLE unset), so the
 * CPU runs at the Kconfig frequency and the cycle counter follows it. */
#define CPU_HZ ((uint32_t)CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ * 1000000u)

#define CLK_HZ_DEFAULT 1000000u
#define CLK_HZ_MIN       50000u
#define CLK_HZ_MAX     4000000u
#define HALF_CYCLES_MIN 20u

static bool claimed;
/* An interrupt that lands inside a transfer stretches a half period far
 * enough that the target loses the request framing; after that the DP stops
 * answering until it is switched to SWD again.  Keep each wire operation
 * electrically contiguous. */
static portMUX_TYPE swd_mux = portMUX_INITIALIZER_UNLOCKED;

static inline void swd_lock(void) {
    portENTER_CRITICAL(&swd_mux);
}

static inline void swd_unlock(void) {
    portEXIT_CRITICAL(&swd_mux);
}
/* The reset wire is only driven when the host explicitly asks for it: on a
 * plain dev board the probe does not own nRESET, and holding that pad as an
 * output can fight whatever the target board wired to it. */
static bool nrst_owned;
static uint32_t clock_hz = CLK_HZ_DEFAULT;
static uint32_t half_cycles;
static uint8_t idle_cycles;
static uint8_t turnaround = 1;
static bool data_phase;

/* Runtime so a diagnostic can try the two wires crossed. */
static int io_pin  = IO_PIN;
static int clk_pin = CLK_PIN;
static int dir_io_pin = DIR_IO;
static int dir_clk_pin = DIR_CLK;

static inline void pin_write(int pin, int level) {
    if (level) {
        GPIO.out_w1ts = 1u << pin;
    }
    else {
        GPIO.out_w1tc = 1u << pin;
    }
}

static inline void pin_out_en(int pin, bool en) {
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
    return (volatile uint32_t *)(IO_MUX_GPIO0_REG + 4u * (uint32_t)pin);
}

/* Route the pad to plain GPIO, make it an output, push-pull.  The input path
 * stays on so a diagnostic can read back what the pad actually sees. */
static void pad_out(int pin) {
    esp_rom_gpio_pad_select_gpio(pin);
    esp_rom_gpio_connect_out_signal(pin, SIG_GPIO_OUT_IDX, false, false);
    pin_out_en(pin, true);
    *mux_reg(pin) |= (1u << 9); /* FUN_IE */
}

/* Route the pad to plain GPIO, release it and enable input + pull-up.
 * UART leaves FUN_IE/FUN_WPU in a state that makes GPIO.in read 0. */
static void pad_in(int pin) {
    esp_rom_gpio_pad_select_gpio(pin);
    esp_rom_gpio_connect_out_signal(pin, SIG_GPIO_OUT_IDX, false, false);
    pin_out_en(pin, false);
    *mux_reg(pin) |= (1u << 8) | (1u << 9); /* FUN_WPU | FUN_IE */
    *mux_reg(pin) &= ~(1u << 7);            /* FUN_WPD off */
}

/* Reset pad: an output we own, or a released input with the board's pull-up
 * so SWJ_Pins can still report its level. */
static void pad_nrst_apply(void) {
    if (nrst_owned) {
        pad_out(NRST_PIN);
        pin_write(NRST_PIN, 1);
    }
    else {
        pad_in(NRST_PIN);
    }
}

/* DIR low = target drives SWDIO, high = host drives it. */
static inline void io_dir(bool out) {
    pin_write(dir_io_pin, out ? 1 : 0);
    pin_out_en(io_pin, out);
}

static inline void dly(void) {
    uint32_t start = esp_cpu_get_cycle_count();

    while ((uint32_t)(esp_cpu_get_cycle_count() - start) < half_cycles) {
    }
}

static inline void clock_cycle(void) {
    pin_write(clk_pin, 0);
    dly();
    pin_write(clk_pin, 1);
    dly();
}

/* Back to idle the way the reference probe does: SWDIO high, SWCLK low, with a
 * half period between releasing SWDIO and driving it again so host and target
 * never drive the line together. */
static inline void drive_idle(void) {
    pin_write(io_pin, 1);
    dly();
    io_dir(true);
    pin_write(clk_pin, 0);
}

static inline void write_bit(int bit) {
    pin_write(io_pin, bit);
    clock_cycle();
}

static inline int read_bit(void) {
    int bit;

    pin_write(clk_pin, 0);
    dly();
    bit = pin_read(io_pin);
    pin_write(clk_pin, 1);
    dly();
    return bit;
}

bool swd_dap_claimed(void) {
    return claimed;
}

void swd_dap_set_clock(uint32_t hz) {
    if (hz < CLK_HZ_MIN) {
        hz = CLK_HZ_MIN;
    }
    if (hz > CLK_HZ_MAX) {
        hz = CLK_HZ_MAX;
    }
    clock_hz = hz;
    half_cycles = CPU_HZ / (2 * hz);
    if (half_cycles < HALF_CYCLES_MIN) {
        half_cycles = HALF_CYCLES_MIN;
    }
}

uint32_t swd_dap_clock(void) {
    return clock_hz;
}

bool swd_dap_claim(void) {
    if (claimed) {
        return true;
    }
#if !defined(CWW_DAP_ONLY)
    /* The ISP clock keeps driving the GPIO4 pad through LEDC. */
    ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 1);
#endif

    pad_nrst_apply();
    pad_out(dir_io_pin);
    pad_out(dir_clk_pin);

    pad_in(io_pin);
    pad_out(clk_pin);
    pin_write(clk_pin, 0);
    pin_write(dir_clk_pin, 1); /* SWCLK is always ESP32 -> target */
    io_dir(true);
    pin_write(io_pin, 1);

    if (half_cycles == 0) {
        swd_dap_set_clock(clock_hz);
    }
    claimed = true;
    return true;
}

void swd_dap_release(void) {
    if (!claimed) {
        return;
    }
    io_dir(true);
    pin_write(io_pin, 1);
    pin_write(clk_pin, 0);
    pin_write(dir_clk_pin, 0);
    claimed = false;
}

/* Swap which pad carries SWDIO and which carries SWCLK, for the diagnostic
 * that checks whether the two wires are crossed. */
void swd_dap_set_swapped(bool on) {
    pin_out_en(io_pin, false);
    pin_out_en(clk_pin, false);
    io_pin = on ? CLK_PIN : IO_PIN;
    clk_pin = on ? IO_PIN : CLK_PIN;
    dir_io_pin = on ? DIR_CLK : DIR_IO;
    dir_clk_pin = on ? DIR_IO : DIR_CLK;
    if (claimed) {
        pad_in(io_pin);
        pad_out(clk_pin);
        pin_write(clk_pin, 0);
        pin_write(dir_clk_pin, 1);
        io_dir(true);
        pin_write(io_pin, 1);
    }
}

bool swd_dap_swapped(void) {
    return io_pin != IO_PIN;
}

/* Release SWDIO to the target without ending the session. */
void swd_dap_line_release(void) {
    if (claimed) {
        io_dir(false);
    }
}

/* pull: 1 = internal pull-up, -1 = internal pull-down, 0 = floating. */
void swd_dap_set_io_pull(int pull) {
    uint32_t m = *mux_reg(io_pin);

    m &= ~((1u << 7) | (1u << 8));
    if (pull > 0) {
        m |= 1u << 8;
    }
    else if (pull < 0) {
        m |= 1u << 7;
    }
    *mux_reg(io_pin) = m;
}

void swd_dap_configure(uint8_t value) {
    turnaround = (uint8_t)((value & 0x03u) + 1u);
    data_phase = (value & 0x04u) != 0;
}

void swd_dap_set_idle(uint8_t cycles) {
    idle_cycles = cycles;
}

void swd_dap_take_nrst(bool own) {
    if (nrst_owned == own) {
        return;
    }
    nrst_owned = own;
    pad_nrst_apply();
}

void swd_dap_nreset(int level) {
    if (!nrst_owned) {
        return;
    }
    pin_write(NRST_PIN, level ? 1 : 0);
}

int swd_dap_nreset_state(void) {
    return pin_read(NRST_PIN);
}

void swd_dap_swj_pins(uint8_t value, uint8_t mask) {
    if (mask & 0x01u) {
        pin_write(clk_pin, (value >> 0) & 1u);
    }
    if (mask & 0x02u) {
        io_dir(true);
        pin_write(io_pin, (value >> 1) & 1u);
    }
    if (mask & 0x80u) {
        swd_dap_take_nrst(true);
        pin_write(NRST_PIN, (value >> 7) & 1u);
    }
}

int swd_dap_pins(void) {
    return (pin_read(clk_pin) << 0) | (pin_read(io_pin) << 1) | (pin_read(NRST_PIN) << 7);
}

void swd_dap_delay_us(uint32_t us) {
    uint32_t cycles = (CPU_HZ / 1000000u) * us;
    uint32_t start = esp_cpu_get_cycle_count();

    while ((uint32_t)(esp_cpu_get_cycle_count() - start) < cycles) {
    }
}

void swd_dap_line_reset(void) {
    swd_lock();
    io_dir(true);
    pin_write(io_pin, 1);
    for (int i = 0; i < 64; i++) {
        clock_cycle();
    }
    pin_write(io_pin, 1);
    pin_write(clk_pin, 0);
    swd_unlock();
}

/* JTAG-to-SWD select, as the reference probe sends it: line reset, 0xE79E,
 * line reset, then idle clocks with SWDIO low. A SWJ-DP powers up in JTAG mode
 * and ignores SWD requests until this has been seen. */
void swd_dap_enter_swd(void) {
    static const uint8_t jtag_to_swd[2] = {0x9E, 0xE7};

    swd_lock();
    swd_dap_line_reset();
    swd_dap_swj_sequence(16, jtag_to_swd);
    swd_dap_line_reset();
    io_dir(true);
    pin_write(io_pin, 0);
    for (int i = 0; i < 8; i++) {
        clock_cycle();
    }
    pin_write(io_pin, 1);
    pin_write(clk_pin, 0);
    swd_unlock();
}

void swd_dap_swj_sequence(uint32_t count, const uint8_t *data) {
    uint32_t val = 0;
    uint32_t n = 0;

    swd_lock();
    io_dir(true);
    while (count--) {
        if (n == 0) {
            val = *data++;
            n = 8;
        }
        write_bit((int)(val & 1u));
        val >>= 1;
        n--;
    }
    pin_write(io_pin, 1);
    pin_write(clk_pin, 0);
    swd_unlock();
}

void swd_dap_swd_sequence(uint32_t info, const uint8_t *swdo, uint8_t *swdi) {
    uint32_t n = info & SWD_DAP_SEQ_CLK;

    if (n == 0) {
        n = 64;
    }
    swd_lock();
    if (info & SWD_DAP_SEQ_DIN) {
        io_dir(false);
        while (n) {
            uint32_t val = 0;
            uint32_t k;

            for (k = 8; k && n; k--, n--) {
                int bit = read_bit();

                val >>= 1;
                val |= (uint32_t)bit << 7;
            }
            val >>= k;
            *swdi++ = (uint8_t)val;
        }
        io_dir(true);
    }
    else {
        io_dir(true);
        while (n) {
            uint32_t val = *swdo++;
            uint32_t k;

            for (k = 8; k && n; k--, n--) {
                write_bit((int)(val & 1u));
                val >>= 1;
            }
        }
    }
    pin_write(io_pin, 1);
    pin_write(clk_pin, 0);
    swd_unlock();
}

uint8_t swd_dap_transfer(uint8_t request, uint32_t *data) {
    uint32_t ack;
    uint32_t val;
    uint32_t parity;
    int n;

    swd_lock();
    parity = 0;
    write_bit(1);
    write_bit(request & 1u);
    parity += request & 1u;
    write_bit((request >> 1) & 1u);
    parity += (request >> 1) & 1u;
    write_bit((request >> 2) & 1u);
    parity += (request >> 2) & 1u;
    write_bit((request >> 3) & 1u);
    parity += (request >> 3) & 1u;
    write_bit((int)(parity & 1u));
    write_bit(0); /* stop */
    write_bit(1); /* park */

    io_dir(false);
    for (n = turnaround; n; n--) {
        clock_cycle();
    }

    ack = (uint32_t)read_bit();
    ack |= (uint32_t)read_bit() << 1;
    ack |= (uint32_t)read_bit() << 2;

    if (ack == SWD_DAP_ACK_OK) {
        if (request & SWD_DAP_RNW) {
            val = 0;
            parity = 0;
            for (n = 32; n; n--) {
                int bit = read_bit();
                parity += (uint32_t)bit;
                val >>= 1;
                val |= (uint32_t)bit << 31;
            }
            if ((parity ^ (uint32_t)read_bit()) & 1u) {
                ack = SWD_DAP_ACK_ERROR;
            }
            if (data) {
                *data = val;
            }
            for (n = turnaround; n; n--) {
                clock_cycle();
            }
            /* Stay released through the turnaround; the host only drives
             * SWDIO again once the target has given the line back. */
            pin_write(io_pin, 1);
        }
        else {
            for (n = turnaround; n; n--) {
                clock_cycle();
            }
            io_dir(true);
            val = data ? *data : 0;
            parity = 0;
            for (n = 32; n; n--) {
                write_bit((int)(val & 1u));
                parity += val & 1u;
                val >>= 1;
            }
            write_bit((int)(parity & 1u));
        }
        if (idle_cycles) {
            io_dir(true);
            pin_write(io_pin, 0);
            for (n = idle_cycles; n; n--) {
                clock_cycle();
            }
        }
        drive_idle();
        swd_unlock();
        return (uint8_t)ack;
    }

    if (ack == SWD_DAP_ACK_WAIT || ack == SWD_DAP_ACK_FAULT) {
        if (data_phase && (request & SWD_DAP_RNW)) {
            for (n = 33; n; n--) {
                clock_cycle();
            }
        }
        for (n = turnaround; n; n--) {
            clock_cycle();
        }
        io_dir(true);
        if (data_phase && !(request & SWD_DAP_RNW)) {
            pin_write(io_pin, 0);
            for (n = 33; n; n--) {
                clock_cycle();
            }
        }
        drive_idle();
        swd_unlock();
        return (uint8_t)ack;
    }

    /* Protocol error: back off the data phase. */
    for (n = turnaround + 33; n; n--) {
        clock_cycle();
    }
    drive_idle();
    swd_unlock();
    return (uint8_t)ack;
}

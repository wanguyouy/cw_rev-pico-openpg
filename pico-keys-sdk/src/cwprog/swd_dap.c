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
#include "esp_clk.h"
#include "esp_cpu.h"
#include "esp_rom_gpio.h"
#include "soc/gpio_sig_map.h"
#include "soc/gpio_struct.h"
#include "soc/io_mux_reg.h"

#define IO_PIN   4  /* PA07 */
#define CLK_PIN  5  /* PA08 */
#define NRST_PIN 6
#define DIR_IO   15 /* U1: GPIO4 <-> PA07 */
#define DIR_CLK  16 /* U2: GPIO5 <-> PA08 */

#define CLK_HZ_DEFAULT 1000000u
#define CLK_HZ_MIN       50000u
#define CLK_HZ_MAX     4000000u
#define HALF_CYCLES_MIN 20u

static bool claimed;
static uint32_t clock_hz = CLK_HZ_DEFAULT;
static uint32_t half_cycles;
static uint8_t idle_cycles;
static uint8_t turnaround = 1;
static bool data_phase;

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
    return pin == IO_PIN ? (volatile uint32_t *)IO_MUX_GPIO4_REG
                         : (volatile uint32_t *)IO_MUX_GPIO5_REG;
}

/* Route the pad to plain GPIO, make it an output, push-pull. */
static void pad_out(int pin) {
    esp_rom_gpio_pad_select_gpio(pin);
    esp_rom_gpio_connect_out_signal(pin, SIG_GPIO_OUT_IDX, false, false);
    pin_out_en(pin, true);
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

/* DIR1 low = target drives SWDIO, high = host drives it. */
static inline void io_dir(bool out) {
    pin_write(DIR_IO, out ? 1 : 0);
    pin_out_en(IO_PIN, out);
}

static inline void dly(void) {
    uint32_t start = esp_cpu_get_cycle_count();

    while ((uint32_t)(esp_cpu_get_cycle_count() - start) < half_cycles) {
    }
}

static inline void clock_cycle(void) {
    pin_write(CLK_PIN, 0);
    dly();
    pin_write(CLK_PIN, 1);
    dly();
}

static inline void write_bit(int bit) {
    pin_write(IO_PIN, bit);
    clock_cycle();
}

static inline int read_bit(void) {
    int bit;

    pin_write(CLK_PIN, 0);
    dly();
    bit = pin_read(IO_PIN);
    pin_write(CLK_PIN, 1);
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
    half_cycles = esp_clk_cpu_freq() / (2 * hz);
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
    /* The ISP clock keeps driving the GPIO4 pad through LEDC. */
    ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 1);

    pad_in(IO_PIN);
    pad_out(CLK_PIN);
    pad_out(NRST_PIN);
    pad_out(DIR_IO);
    pad_out(DIR_CLK);

    pin_write(CLK_PIN, 0);
    pin_write(NRST_PIN, 1);
    pin_write(DIR_CLK, 1); /* SWCLK is always ESP32 -> target */
    pin_write(DIR_IO, 1);
    pin_write(IO_PIN, 1);

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
    pin_write(IO_PIN, 1);
    pin_write(CLK_PIN, 0);
    pin_write(DIR_CLK, 0);
    claimed = false;
}

void swd_dap_configure(uint8_t value) {
    turnaround = (uint8_t)((value & 0x03u) + 1u);
    data_phase = (value & 0x04u) != 0;
}

void swd_dap_set_idle(uint8_t cycles) {
    idle_cycles = cycles;
}

void swd_dap_nreset(int level) {
    pin_write(NRST_PIN, level ? 1 : 0);
}

int swd_dap_nreset_state(void) {
    return pin_read(NRST_PIN);
}

void swd_dap_swj_pins(uint8_t value, uint8_t mask) {
    if (mask & 0x01u) {
        pin_write(CLK_PIN, (value >> 0) & 1u);
    }
    if (mask & 0x02u) {
        io_dir(true);
        pin_write(IO_PIN, (value >> 1) & 1u);
    }
    if (mask & 0x80u) {
        pin_write(NRST_PIN, (value >> 7) & 1u);
    }
}

int swd_dap_pins(void) {
    return (pin_read(CLK_PIN) << 0) | (pin_read(IO_PIN) << 1) | (pin_read(NRST_PIN) << 7);
}

void swd_dap_delay_us(uint32_t us) {
    uint32_t cycles = (esp_clk_cpu_freq() / 1000000u) * us;
    uint32_t start = esp_cpu_get_cycle_count();

    while ((uint32_t)(esp_cpu_get_cycle_count() - start) < cycles) {
    }
}

void swd_dap_line_reset(void) {
    io_dir(true);
    pin_write(IO_PIN, 1);
    for (int i = 0; i < 56; i++) {
        clock_cycle();
    }
    pin_write(IO_PIN, 1);
}

void swd_dap_swj_sequence(uint32_t count, const uint8_t *data) {
    uint32_t val = 0;
    uint32_t n = 0;

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
    pin_write(IO_PIN, 1);
}

void swd_dap_swd_sequence(uint32_t info, const uint8_t *swdo, uint8_t *swdi) {
    uint32_t n = info & SWD_DAP_SEQ_CLK;

    if (n == 0) {
        n = 64;
    }
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
}

uint8_t swd_dap_transfer(uint8_t request, uint32_t *data) {
    uint32_t ack;
    uint32_t val;
    uint32_t parity;
    int n;

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
            io_dir(true);
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
            pin_write(IO_PIN, 0);
            for (n = idle_cycles; n; n--) {
                clock_cycle();
            }
        }
        pin_write(IO_PIN, 1);
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
            pin_write(IO_PIN, 0);
            for (n = 33; n; n--) {
                clock_cycle();
            }
        }
        pin_write(IO_PIN, 1);
        return (uint8_t)ack;
    }

    /* Protocol error: back off the data phase. */
    for (n = turnaround + 33; n; n--) {
        clock_cycle();
    }
    io_dir(true);
    pin_write(IO_PIN, 1);
    return (uint8_t)ack;
}

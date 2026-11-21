/*
 * CMSIS-DAP SWD engine on the CW Writer pads (GPIO4/5/6 + DIR on GPIO15/16).
 *
 * Port of ARM's SW_DP.c (DAPLink, Apache-2.0) onto the register-level pad
 * control already used by swd.c: UART1 reserves GPIO4/5, so the bit-bang
 * path re-muxes the pads to plain GPIO and drives them directly.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#define SWD_DAP_RNW         0x02u
#define SWD_DAP_APNDP       0x01u
#define SWD_DAP_MATCH_VALUE 0x10u
#define SWD_DAP_MATCH_MASK  0x20u
#define SWD_DAP_TIMESTAMP   0x80u

#define SWD_DAP_ACK_OK    0x01u
#define SWD_DAP_ACK_WAIT  0x02u
#define SWD_DAP_ACK_FAULT 0x04u
#define SWD_DAP_ACK_ERROR 0x08u

#define SWD_DAP_SEQ_DIN 0x80u
#define SWD_DAP_SEQ_CLK 0x3Fu

/* Takes the pads from the ISP link (stops the LEDC clock, re-muxes
 * GPIO4/5, drives DIR1=1 / DIR2=1, releases NRST).  Idempotent. */
bool swd_dap_claim(void);
void swd_dap_release(void);
bool swd_dap_claimed(void);

/* Clock is the SWCLK frequency, clamped to [50 kHz, 4 MHz]. */
void swd_dap_set_clock(uint32_t hz);
uint32_t swd_dap_clock(void);

void swd_dap_configure(uint8_t value); /* bit0-1 turnaround-1, bit2 data phase */
void swd_dap_set_idle(uint8_t cycles);

/* One DP/AP transaction.  request = APnDP|RnW|A2|A3.  Returns the raw
 * ACK: OK=1, WAIT=2, FAULT=4, ERROR=8 (parity error or protocol error). */
uint8_t swd_dap_transfer(uint8_t request, uint32_t *data);

/* count bits, LSB first out of data (SWJ_Sequence / 0xE79E JTAG-to-SWD). */
void swd_dap_swj_sequence(uint32_t count, const uint8_t *data);

/* count = info & 0x3F (0 means 64); with DIN set the bits are captured. */
void swd_dap_swd_sequence(uint32_t info, const uint8_t *swdo, uint8_t *swdi);

void swd_dap_line_reset(void);

/* level 0 asserts nRESET (as cw_isp_entry() and swd.c do). */
void swd_dap_nreset(int level);
int swd_dap_nreset_state(void);

/* Drives the pins selected in mask (bit0 SWCLK, bit1 SWDIO, bit7 nRESET). */
void swd_dap_swj_pins(uint8_t value, uint8_t mask);

/* Pin levels: bit0 SWCLK, bit1 SWDIO, bit7 nRESET. */
int swd_dap_pins(void);

void swd_dap_delay_us(uint32_t us);

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Prepares the CW32 ISP link (UART1 + LEDC + NRST). Call once at start-up. */
void cw_isp_prog_start(void);

/* Sends one raw ISP frame (0x65 len body crc) to the real target and copies
 * the target's answer frame back into rx.  Entering the bootloader is handled
 * automatically, including after a Jump.  Returns 0 on success. */
int cw_isp_link_xfer(const uint8_t *tx, uint16_t txlen, uint8_t *rx, uint16_t rxcap,
                     uint16_t *rxlen);

/* 只做一次普通复位（不灌 50kHz 方波）后直接发 Query。
 * 若仍能应答，说明芯片复位后本来就停在 BootLoader 里。返回 0 表示有应答。 */
int cw_isp_query_cold(uint8_t *rx, uint16_t rxcap, uint16_t *rxlen);

/* The SWD/DAP session takes PA07/PA08 over; ISP transfers are refused while
 * the pads are held.  Release restores UART1 and the level-shifter DIRs. */
void cw_isp_pads_hold_for_swd(void);
void cw_isp_pads_release_from_swd(void);
bool cw_isp_pads_held(void);

#ifdef __cplusplus
}
#endif

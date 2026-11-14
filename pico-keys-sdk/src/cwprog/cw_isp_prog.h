#pragma once

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

#ifdef __cplusplus
}
#endif

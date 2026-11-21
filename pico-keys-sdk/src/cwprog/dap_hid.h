/*
 * CMSIS-DAP v1 (HID, 64-byte reports) over the CW Writer SWD pads.
 * Command/response layout follows ARM DAPLink's DAP.c.
 */

#pragma once

#include <stdbool.h>

/* True while a debugger owns the SWD pads. */
bool dap_hid_connected(void);

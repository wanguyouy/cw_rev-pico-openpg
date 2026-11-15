/*
 * Status display for the SSD1306 OLED of the CW32L010 ISP bridge.
 *
 * The screen is read-only: values are pushed by the application (or a later
 * PC / web control channel), never navigated locally.
 */

#ifndef UI_DISPLAY_H
#define UI_DISPLAY_H

#include <stdbool.h>

typedef enum {
    UI_MODE_ISP = 0,
    UI_MODE_SWD
} ui_mode_t;

void ui_init(void);
void ui_refresh(void);

/* Throttled redraw; call from the main loop.  Setters only mark the screen
 * dirty so callers on the USB/ISP path never block on I2C. */
void ui_task(void);

void ui_set_mode(ui_mode_t mode);
/* 供电/复位方式，如 "3.3V/RST"、"SELF/SWD" */
void ui_set_target(const char *text);
void ui_set_file(const char *name);
void ui_set_chip(const char *name);
void ui_set_status(const char *status);

#endif /* UI_DISPLAY_H */

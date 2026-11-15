/*
 * Minimal SSD1306 128x64 I2C driver for the CW32L010 ISP bridge.
 *
 * SDA = GPIO8, SCL = GPIO9 (I2C address 0x3C).
 */

#ifndef SSD1306_H
#define SSD1306_H

#include <stdbool.h>
#include <stdint.h>

#define SSD1306_WIDTH  128
#define SSD1306_HEIGHT 64

void ssd1306_init(void);
/* True when a panel answered on the bus during ssd1306_init(). */
bool ssd1306_present(void);
/* 7-bit address of the detected panel (0 when absent). */
uint8_t ssd1306_addr(void);
void ssd1306_clear(void);
void ssd1306_set_pixel(uint8_t x, uint8_t y, bool on);
void ssd1306_fill_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, bool on);
/* 5x7 font, 6 px advance. y is in pixels. */
void ssd1306_draw_text(uint8_t x, uint8_t y, const char *text, bool invert);
void ssd1306_flush(void);

#endif /* SSD1306_H */

/*
 * Status display for the SSD1306 OLED of the CW32L010 ISP bridge.
 */

#include "ui_display.h"

#include <stdio.h>
#include <string.h>

#include "ssd1306.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define UI_REFRESH_MS 250

#define TITLE_Y     0
#define RULE_TOP_Y  9
#define ROW0_Y      12
#define ROW_STEP    9
#define RULE_BOT_Y  48
#define STATUS_Y    53

#define FIELD_LEN   20

static ui_mode_t mode      = UI_MODE_ISP;
static char      target_text[14] = "SELF/SWD";
static char      file_name[FIELD_LEN] = "---";
static char      chip_name[FIELD_LEN] = "---";
static char      status_text[22]      = "READY";

static volatile bool dirty;
static uint32_t last_refresh_ms;

static void set_text(char *dst, size_t size, const char *src) {
    if (src == NULL) {
        src = "---";
    }
    strncpy(dst, src, size - 1);
    dst[size - 1] = '\0';
}

static void draw_info(uint8_t row, const char *label, const char *value) {
    char line[24];
    snprintf(line, sizeof(line), "%-5s %s", label, value);
    ssd1306_draw_text(0, (uint8_t)(ROW0_Y + row * ROW_STEP), line, false);
}

void ui_refresh(void) {
    ssd1306_clear();
    ssd1306_draw_text(19, TITLE_Y, "CW32L010 BRIDGE", false);
    ssd1306_fill_rect(0, RULE_TOP_Y, SSD1306_WIDTH, 1, true);
    draw_info(0, "MODE", mode == UI_MODE_SWD ? "SWD" : "ISP");
    draw_info(1, "PWR", target_text);
    draw_info(2, "FILE", file_name);
    draw_info(3, "CHIP", chip_name);
    ssd1306_fill_rect(0, RULE_BOT_Y, SSD1306_WIDTH, 1, true);
    ssd1306_draw_text(1, STATUS_Y, status_text, false);
    ssd1306_flush();
}

void ui_init(void) {
    ssd1306_init();
    mode = UI_MODE_ISP;
    set_text(target_text, sizeof(target_text), "SELF/SWD");
    set_text(file_name, sizeof(file_name), "---");
    set_text(chip_name, sizeof(chip_name), "---");
    if (!ssd1306_present()) {
        set_text(status_text, sizeof(status_text), "OLED NG");
    }
    last_refresh_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    ui_refresh();
}

void ui_task(void) {
    if (!dirty) {
        return;
    }
    uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    if ((now - last_refresh_ms) < UI_REFRESH_MS) {
        return;
    }
    last_refresh_ms = now;
    dirty = false;
    ui_refresh();
}

void ui_set_mode(ui_mode_t value) {
    mode = value;
    dirty = true;
}

void ui_set_target(const char *text) {
    set_text(target_text, sizeof(target_text), text);
    dirty = true;
}

void ui_set_file(const char *name) {
    set_text(file_name, sizeof(file_name), name);
    dirty = true;
}

void ui_set_chip(const char *name) {
    set_text(chip_name, sizeof(chip_name), name);
    dirty = true;
}

void ui_set_status(const char *status) {
    set_text(status_text, sizeof(status_text), status);
    dirty = true;
}

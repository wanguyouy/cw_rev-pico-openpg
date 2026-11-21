/*
 * Minimal SSD1306 128x64 I2C driver for the CW32L010 ISP bridge.
 *
 * SDA = GPIO8, SCL = GPIO9 (I2C address 0x3C), 400 kHz.
 * The frame buffer is page oriented: fb[page][column], bit0 = top row of page.
 */

#include "ssd1306.h"

#include <string.h>

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define SSD1306_SDA_GPIO 8
#define SSD1306_SCL_GPIO 9
#define SSD1306_I2C_ADDR 0x3C /* 8-bit write address 0x78 */
#define SSD1306_I2C_ADDR_ALT 0x3D
#define SSD1306_I2C_HZ   100000

#define SSD1306_CTRL_CMD  0x00
#define SSD1306_CTRL_DATA 0x40

static const char *TAG = "ssd1306";

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_dev;
static uint8_t s_addr;
static uint8_t s_fb[SSD1306_HEIGHT / 8][SSD1306_WIDTH];

/* 5x7 glyphs, column major, bit0 = top row. Index into FONT_INDEX. */
static const char FONT_INDEX[] = " 0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ.:-/%_";

static const uint8_t FONT[][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, /* ' ' */
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, /* '0' */
    {0x00, 0x42, 0x7F, 0x40, 0x00}, /* '1' */
    {0x42, 0x61, 0x51, 0x49, 0x46}, /* '2' */
    {0x21, 0x41, 0x45, 0x4B, 0x31}, /* '3' */
    {0x18, 0x14, 0x12, 0x7F, 0x10}, /* '4' */
    {0x27, 0x45, 0x45, 0x45, 0x39}, /* '5' */
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, /* '6' */
    {0x01, 0x71, 0x09, 0x05, 0x03}, /* '7' */
    {0x36, 0x49, 0x49, 0x49, 0x36}, /* '8' */
    {0x06, 0x49, 0x49, 0x29, 0x1E}, /* '9' */
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, /* 'A' */
    {0x7F, 0x49, 0x49, 0x49, 0x36}, /* 'B' */
    {0x3E, 0x41, 0x41, 0x41, 0x22}, /* 'C' */
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, /* 'D' */
    {0x7F, 0x49, 0x49, 0x49, 0x41}, /* 'E' */
    {0x7F, 0x09, 0x09, 0x09, 0x01}, /* 'F' */
    {0x3E, 0x41, 0x49, 0x49, 0x7A}, /* 'G' */
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, /* 'H' */
    {0x00, 0x41, 0x7F, 0x41, 0x00}, /* 'I' */
    {0x20, 0x40, 0x41, 0x3F, 0x01}, /* 'J' */
    {0x7F, 0x08, 0x14, 0x22, 0x41}, /* 'K' */
    {0x7F, 0x40, 0x40, 0x40, 0x40}, /* 'L' */
    {0x7F, 0x02, 0x0C, 0x02, 0x7F}, /* 'M' */
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, /* 'N' */
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, /* 'O' */
    {0x7F, 0x09, 0x09, 0x09, 0x06}, /* 'P' */
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, /* 'Q' */
    {0x7F, 0x09, 0x19, 0x29, 0x46}, /* 'R' */
    {0x46, 0x49, 0x49, 0x49, 0x31}, /* 'S' */
    {0x01, 0x01, 0x7F, 0x01, 0x01}, /* 'T' */
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, /* 'U' */
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, /* 'V' */
    {0x3F, 0x40, 0x38, 0x40, 0x3F}, /* 'W' */
    {0x63, 0x14, 0x08, 0x14, 0x63}, /* 'X' */
    {0x07, 0x08, 0x70, 0x08, 0x07}, /* 'Y' */
    {0x61, 0x51, 0x49, 0x45, 0x43}, /* 'Z' */
    {0x00, 0x60, 0x60, 0x00, 0x00}, /* '.' */
    {0x00, 0x36, 0x36, 0x00, 0x00}, /* ':' */
    {0x08, 0x08, 0x08, 0x08, 0x08}, /* '-' */
    {0x20, 0x10, 0x08, 0x04, 0x02}, /* '/' */
    {0x62, 0x64, 0x08, 0x13, 0x23}, /* '%' */
    {0x40, 0x40, 0x40, 0x40, 0x40}, /* '_' */
};

static const uint8_t INIT_CMDS[] = {
    0xAE,       /* display off */
    0xD5, 0x80, /* clock divide */
    0xA8, 0x3F, /* multiplex 1/64 */
    0xD3, 0x00, /* display offset */
    0x40,       /* start line 0 */
    0x8D, 0x14, /* charge pump on */
    0x20, 0x00, /* horizontal addressing */
    0xA1,       /* segment remap */
    0xC8,       /* COM scan dec */
    0xDA, 0x12, /* COM pins */
    0x81, 0xCF, /* contrast */
    0xD9, 0xF1, /* pre-charge */
    0xDB, 0x40, /* VCOMH */
    0xA4,       /* display follows RAM */
    0xA6,       /* normal (not inverted) */
    0xAF,       /* display on */
};

static void ssd1306_write_cmd(const uint8_t *cmds, size_t len) {
    uint8_t buf[32];
    if (s_dev == NULL || len + 1 > sizeof(buf)) {
        return;
    }
    buf[0] = SSD1306_CTRL_CMD;
    memcpy(buf + 1, cmds, len);
    if (i2c_master_transmit(s_dev, buf, len + 1, 100) != ESP_OK) {
        ESP_LOGW(TAG, "command transfer failed");
    }
}

static bool open_bus(int sda, int scl, uint8_t addr) {
    i2c_master_bus_handle_t bus = NULL;
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = -1,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    if (i2c_new_master_bus(&bus_cfg, &bus) != ESP_OK) {
        return false;
    }
    /* The first transaction after the bus is created times out on this board
     * (the lines need a moment to settle with the weak internal pull-ups),
     * so probe the address a few times before declaring it absent. */
    bool acked = false;
    for (int i = 0; i < 5 && !acked; i++) {
        if (i) {
            vTaskDelay(pdMS_TO_TICKS(2));
        }
        acked = i2c_master_probe(bus, addr, 20) == ESP_OK;
    }
    if (!acked) {
        i2c_del_master_bus(bus);
        return false;
    }
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = SSD1306_I2C_HZ,
    };
    i2c_master_dev_handle_t dev = NULL;
    if (i2c_master_bus_add_device(bus, &dev_cfg, &dev) != ESP_OK) {
        i2c_del_master_bus(bus);
        return false;
    }
    s_bus = bus;
    s_dev = dev;
    s_addr = addr;
    ESP_LOGI(TAG, "panel found: sda=%d scl=%d addr=0x%02X", sda, scl, addr);
    return true;
}

static void scan_bus(int sda, int scl) {
    i2c_master_bus_handle_t bus = NULL;
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = -1,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    if (i2c_new_master_bus(&bus_cfg, &bus) != ESP_OK) {
        ESP_LOGE(TAG, "scan: bus init failed (sda=%d scl=%d)", sda, scl);
        return;
    }
    int found = 0;
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        if (i2c_master_probe(bus, addr, 10) == ESP_OK) {
            ESP_LOGW(TAG, "scan: device at 0x%02X (sda=%d scl=%d)", addr, sda, scl);
            found++;
        }
    }
    i2c_del_master_bus(bus);
    ESP_LOGW(TAG, "scan: %d device(s) on sda=%d scl=%d", found, sda, scl);
}

void ssd1306_init(void) {
    if (s_dev != NULL) {
        return;
    }
    /* VDD has to settle before the controller accepts commands. */
    vTaskDelay(pdMS_TO_TICKS(200));

    /* Address first, then swapped data/clock lines for wiring mistakes.  The
     * controller NACKs for tens of ms after VDD comes up, so sweep the
     * combinations a few times instead of giving up on the first pass. */
    static const uint8_t addrs[2] = {SSD1306_I2C_ADDR, SSD1306_I2C_ADDR_ALT};
    for (int round = 0; round < 4 && s_dev == NULL; round++) {
        for (int attempt = 0; attempt < 4; attempt++) {
            int sda = (attempt < 2) ? SSD1306_SDA_GPIO : SSD1306_SCL_GPIO;
            int scl = (attempt < 2) ? SSD1306_SCL_GPIO : SSD1306_SDA_GPIO;
            if (open_bus(sda, scl, addrs[attempt & 1])) {
                break;
            }
        }
        if (s_dev == NULL) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
    if (s_dev == NULL) {
        ESP_LOGE(TAG, "no panel answered (tried 0x%02X/0x%02X, both pin orders)",
                 SSD1306_I2C_ADDR, SSD1306_I2C_ADDR_ALT);
        scan_bus(SSD1306_SDA_GPIO, SSD1306_SCL_GPIO);
        scan_bus(SSD1306_SCL_GPIO, SSD1306_SDA_GPIO);
        return;
    }
    ssd1306_write_cmd(INIT_CMDS, sizeof(INIT_CMDS));
    ssd1306_clear();
    ssd1306_flush();
}

bool ssd1306_present(void) {
    return s_dev != NULL;
}

uint8_t ssd1306_addr(void) {
    return s_addr;
}

void ssd1306_clear(void) {
    memset(s_fb, 0, sizeof(s_fb));
}

void ssd1306_set_pixel(uint8_t x, uint8_t y, bool on) {
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) {
        return;
    }
    uint8_t mask = (uint8_t)(1u << (y & 7));
    if (on) {
        s_fb[y >> 3][x] |= mask;
    }
    else {
        s_fb[y >> 3][x] &= (uint8_t)~mask;
    }
}

void ssd1306_fill_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, bool on) {
    for (uint8_t dy = 0; dy < h && (y + dy) < SSD1306_HEIGHT; dy++) {
        for (uint8_t dx = 0; dx < w && (x + dx) < SSD1306_WIDTH; dx++) {
            ssd1306_set_pixel((uint8_t)(x + dx), (uint8_t)(y + dy), on);
        }
    }
}

static int glyph_index(char c) {
    if (c >= 'a' && c <= 'z') {
        c = (char)(c - 'a' + 'A');
    }
    for (size_t i = 0; FONT_INDEX[i] != '\0'; i++) {
        if (FONT_INDEX[i] == c) {
            return (int)i;
        }
    }
    return 0; /* space */
}

void ssd1306_draw_text(uint8_t x, uint8_t y, const char *text, bool invert) {
    if (text == NULL) {
        return;
    }
    for (; *text != '\0'; text++) {
        int idx = glyph_index(*text);
        for (uint8_t col = 0; col < 5; col++) {
            uint8_t bits = FONT[idx][col];
            for (uint8_t row = 0; row < 7; row++) {
                bool on = ((bits >> row) & 1u) != 0;
                ssd1306_set_pixel((uint8_t)(x + col), (uint8_t)(y + row), on != invert);
            }
        }
        if (invert) {
            for (uint8_t row = 0; row < 8; row++) {
                ssd1306_set_pixel((uint8_t)(x + 5), (uint8_t)(y + row), true);
            }
        }
        x = (uint8_t)(x + 6);
        if (x >= SSD1306_WIDTH) {
            return;
        }
    }
}

void ssd1306_flush(void) {
    if (s_dev == NULL) {
        return;
    }
    uint8_t payload[SSD1306_WIDTH + 1];
    payload[0] = SSD1306_CTRL_DATA;
    for (uint8_t page = 0; page < SSD1306_HEIGHT / 8; page++) {
        uint8_t cmds[3] = {(uint8_t)(0xB0 | page), 0x00, 0x10};
        ssd1306_write_cmd(cmds, sizeof(cmds));
        memcpy(payload + 1, s_fb[page], SSD1306_WIDTH);
        if (i2c_master_transmit(s_dev, payload, sizeof(payload), 100) != ESP_OK) {
            ESP_LOGW(TAG, "page transfer failed");
            return;
        }
    }
}

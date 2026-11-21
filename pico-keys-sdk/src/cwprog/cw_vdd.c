/*
 * Target VDD rail + reset strategy for the CW32 ISP bridge.
 *
 * Hardware (hardware/design.md): GPIO7 is the master enable of both load
 * switches (active low), GPIO17 selects 3.3V (0) or 5.0V (1), GPIO1 sits on
 * a 1/2 divider across VDD_TGT so an externally powered target can be seen
 * before a rail is switched on.
 */

#include "cw_vdd.h"

#include <stdio.h>

#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ui_display.h"

#define CW_VDD_EN_PIN  GPIO_NUM_7  /* 低有效 */
#define CW_VDD_SEL_PIN GPIO_NUM_17 /* 0 = 3.3V, 1 = 5.0V */
#define CW_VDD_ADC_CH  ADC_CHANNEL_0 /* GPIO1 = ADC1_CH0 */

/* 分压后超过这个值就认为目标板自己已经带电 */
#define CW_VDD_EXT_MV 400

/* 上电后等待目标电压稳定 */
#define CW_VDD_SETTLE_MS 30

static cw_vdd_t   vdd     = CW_VDD_OFF;
static cw_vdd_t   cfg_vdd = CW_VDD_OFF;
static cw_reset_t cfg_rst = CW_RST_SWD;
static adc_oneshot_unit_handle_t adc;

static void vdd_update_ui(void) {
    static const char *const pwr[] = {"SELF", "3.3V", "5.0V"};
    static const char *const rst[] = {"SWD", "PWR", "RST"};
    char text[16];

    snprintf(text, sizeof(text), "%s/%s", pwr[cfg_vdd], rst[cfg_rst]);
    ui_set_target(text);
}

void cw_vdd_init(void) {
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << CW_VDD_EN_PIN) | (1ULL << CW_VDD_SEL_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);

    /* 先断开两条轨再选电压，避免切换瞬间两轨同时导通 */
    gpio_set_level(CW_VDD_EN_PIN, 1);
    gpio_set_level(CW_VDD_SEL_PIN, 0);
    vdd = CW_VDD_OFF;

    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    if (adc_oneshot_new_unit(&unit_cfg, &adc) == ESP_OK) {
        adc_oneshot_chan_cfg_t chan_cfg = {
            .atten = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        adc_oneshot_config_channel(adc, CW_VDD_ADC_CH, &chan_cfg);
    }
    else {
        adc = NULL;
    }

    printf("[cwvdd] rail off, default %s\n", cfg_vdd == CW_VDD_OFF ? "self-powered" : "3.3V");
    vdd_update_ui();
}

int cw_vdd_target_mv(void) {
    int raw = 0;

    if (adc == NULL) {
        return 0;
    }
    if (adc_oneshot_read(adc, CW_VDD_ADC_CH, &raw) != ESP_OK) {
        return 0;
    }
    return raw * 2; /* 1/2 分压 */
}

bool cw_vdd_external(void) {
    return cw_vdd_target_mv() > CW_VDD_EXT_MV;
}

int cw_vdd_adc_mv(int channel) {
    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    int raw = 0;

    if (adc == NULL) {
        return -1;
    }
    if (adc_oneshot_config_channel(adc, (adc_channel_t)channel, &chan_cfg) != ESP_OK ||
        adc_oneshot_read(adc, (adc_channel_t)channel, &raw) != ESP_OK) {
        return -1;
    }
    return raw * 3300 / 4095;
}

cw_vdd_t cw_vdd_get(void) {
    return vdd;
}

static void vdd_switch(cw_vdd_t v, bool check_external) {
    if (v == CW_VDD_OFF) {
        gpio_set_level(CW_VDD_EN_PIN, 1);
        vdd = CW_VDD_OFF;
        return;
    }

    if (check_external && cw_vdd_external()) {
        /* 目标已有电源：再供电就是倒灌 */
        gpio_set_level(CW_VDD_EN_PIN, 1);
        vdd = CW_VDD_OFF;
        printf("[cwvdd] target already powered (%d mV), rail not enabled\n",
               cw_vdd_target_mv());
        ui_set_status("TGT POWERED");
        return;
    }

    gpio_set_level(CW_VDD_SEL_PIN, v == CW_VDD_5V ? 1 : 0);
    gpio_set_level(CW_VDD_EN_PIN, 0);
    vdd = v;
    vTaskDelay(pdMS_TO_TICKS(CW_VDD_SETTLE_MS));
}

void cw_vdd_set(cw_vdd_t v) {
    vdd_switch(v, true);
}

/* For the SWD path: the level shifters need VDD_TGT, and there is no tool APDU
 * to pick a supply there.  The rail switches block reverse current. */
void cw_vdd_force(cw_vdd_t v) {
    vdd_switch(v, false);
}

void cw_vdd_configure(cw_vdd_t v, cw_reset_t r) {
    cfg_vdd = v;
    cfg_rst = r;
    printf("[cwvdd] tool setting: supply=%u reset=%u\n", (unsigned)v, (unsigned)r);
    vdd_update_ui();
    cw_vdd_apply();
}

cw_vdd_t cw_vdd_configured(void) {
    return cfg_vdd;
}

cw_reset_t cw_vdd_reset_mode(void) {
    return cfg_rst;
}

void cw_vdd_apply(void) {
    if (cfg_vdd == CW_VDD_OFF) {
        if (vdd != CW_VDD_OFF) {
            cw_vdd_set(CW_VDD_OFF);
        }
        return;
    }
    if (vdd != cfg_vdd) {
        cw_vdd_set(CW_VDD_OFF);
        vTaskDelay(pdMS_TO_TICKS(10));
        cw_vdd_set(cfg_vdd);
    }
}

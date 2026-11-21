/*
 * Target supply rail and reset strategy, driven by the vendor tool's
 * "编程器供电3.3V/5.0V/目标自供电 + SWD/上电/RST复位" selector.
 */

#pragma once

#include <stdbool.h>

typedef enum {
    CW_VDD_OFF = 0, /* 目标芯片自供电，本机两条轨都断开 */
    CW_VDD_3V3 = 1,
    CW_VDD_5V  = 2,
} cw_vdd_t;

typedef enum {
    CW_RST_SWD   = 0, /* ISP 模式下没有 SWD，退化为拉 NRST */
    CW_RST_POWER = 1, /* 断电再上电；目标自供电时退化为拉 NRST */
    CW_RST_NRST  = 2,
} cw_reset_t;

void cw_vdd_init(void);

/* 立即动作，含倒灌检查（目标已有外部电压时不使能） */
void cw_vdd_set(cw_vdd_t v);
/* 不做倒灌检查（SWD 连接时用：电平转换需要 VDD_TGT 才能工作） */
void cw_vdd_force(cw_vdd_t v);
cw_vdd_t cw_vdd_get(void);

/* 目标 VDD 引脚电压（mV，已按 1/2 分压还原） */
int cw_vdd_target_mv(void);
/* ADC1 任意通道的原始电压（mV，未校准；诊断用） */
int cw_vdd_adc_mv(int channel);
bool cw_vdd_external(void);

/* 工具下发的连接参数（0xF5） */
void cw_vdd_configure(cw_vdd_t v, cw_reset_t r);
cw_vdd_t cw_vdd_configured(void);
cw_reset_t cw_vdd_reset_mode(void);

/* 进入 ISP 前按配置给电/断电，已经到位则不重复动作 */
void cw_vdd_apply(void);

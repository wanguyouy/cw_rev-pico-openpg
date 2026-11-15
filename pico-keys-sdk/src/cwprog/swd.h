/*
 * Bit-banged SWD master for CW32L010 bring-up.
 *
 * The SWD pins are the same pads the ISP bootloader uses:
 *   GPIO4 = PA07, GPIO5 = PA08, GPIO6 = NRST.
 * Which of PA07/PA08 is SWDIO is not documented clearly, so the mapping
 * can be swapped at runtime.  Taking over the pads kills the UART1 ISP
 * link until reset.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

/* swap_pins = 0: SWDIO=GPIO4 (PA07), SWCLK=GPIO5 (PA08)
 * swap_pins = 1: SWDIO=GPIO5 (PA08), SWCLK=GPIO4 (PA07) */
bool swd_init(int swap_pins);

/* 0 = no answer.  Re-inits the link on first use (SWDIO=GPIO4 mapping). */
uint32_t swd_idcode(void);

/* 最近一次事务的 ACK 原始值：1=OK 2=WAIT 4=FAULT 7=线悬空 0=被拉低 */
int swd_last_ack(void);

/* 空闲时 SWDIO 的实际电平（-1 = 未初始化） */
int swd_idle_level(void);

/* 焊盘自检：返回 (驱动1时读回<<4) | (驱动0时读回)，正常为 0x1010 */
int swd_pad_check(void);

/* Prints the probe result over the CDC console. 0 on success. */
int swd_probe(void);

/* 两种引脚映射各试一遍，把结果写回四个出参，结束后恢复 swap_pins=0。 */
void swd_probe_map(uint32_t *id_a, int *ack_a, uint32_t *id_b, int *ack_b);

/* 复位一次后只读采样：bit1 = PA07, bit0 = PA08。
 * PA08 被拉不低说明芯片仍停在 ISP 引导程序（TXD 在驱动高）。 */
int swd_pins_after_reset(void);

/* 不复位，只读采样：bit3..0 = PA07下拉, PA08下拉, PA07上拉, PA08上拉 */
int swd_pins_sample(void);

/* 线上的裸位流：发完 0xA5 读 IDCODE 请求后，连续采 nbits(<=40) 个时钟，
 * 把每次上升沿读到的电平按位拼成一个 64 位值（bit0 = 第一个采样）。
 * 全 1 说明目标完全没在驱动 SWDIO；出现 0 说明有应答但时序错位。 */
uint64_t swd_raw_read(int nbits);

/* 自环测试：GPIO4 驱动时读 GPIO5，再 GPIO5 驱动时读 GPIO4。
 * 返回值 bit3..0 = 4高→5读, 4低→5读, 5高→4读, 5低→4读，正常 0b1010。
 * 需要先把两条线从目标上摘下来短接在一起。 */
int swd_loopback(void);

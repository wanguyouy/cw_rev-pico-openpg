/*
 * Firmware upload over the CDC ACM interface.
 */

#include "fw_upload.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "cw_isp_prog.h"
#include "cw_vdd.h"
#include "fw_store.h"
#include "swd.h"
#include "tusb.h"
#include "ui_display.h"

/* ISP 帧用的 CRC16/X25 */
static uint16_t isp_crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 1) ? (uint16_t)((crc >> 1) ^ 0x8408) : (uint16_t)(crc >> 1);
        }
    }
    return (uint16_t)(crc ^ 0xFFFF);
}

#define RX_MAX 1400

static const char *TAG = "fwupload";

static uint8_t rx[RX_MAX];
static size_t rx_len;
static uint32_t upload_total;
static uint32_t upload_done;

static uint32_t le32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

static void consume(size_t n) {
    if (n >= rx_len) {
        rx_len = 0;
        return;
    }
    memmove(rx, rx + n, rx_len - n);
    rx_len -= n;
}

static void reply(const char *text, size_t len) {
    if (tud_cdc_connected()) {
        tud_cdc_write(text, len);
        tud_cdc_write_flush();
    }
}

void fw_upload_task(void) {
    if (!tud_cdc_connected()) {
        return;
    }
    int avail = (int)tud_cdc_available();
    if (avail > 0) {
        size_t space = sizeof(rx) - rx_len;
        if ((size_t)avail > space) {
            avail = (int)space;
        }
        if (avail > 0) {
            rx_len += tud_cdc_read(rx + rx_len, (size_t)avail);
        }
    }

    for (;;) {
        if (rx_len < 1) {
            return;
        }
        uint8_t cmd = rx[0];

        if (cmd == 'Q') {
            char out[64];
            int n = snprintf(out, sizeof(out), "CWU1 %u %08X %s\n",
                             (unsigned)fw_store_size(), (unsigned)fw_store_auth(),
                             fw_store_valid() ? fw_store_name() : "-");
            reply(out, (size_t)n);
            consume(1);
            continue;
        }

        if (cmd == 'X') {
            /* ISP 整片擦除，把目标恢复成出/空片，便于验证 SWD */
            uint8_t tx[13] = {0x65, 0x09, 0x24, 0xFF, 0xFF, 0xFF, 0xFF,
                              0xFF, 0xFF, 0xFF, 0xFF, 0, 0};
            uint8_t rx[32] = {0};
            uint16_t rxlen = 0;
            uint16_t crc = isp_crc16(tx, 11);
            char out[96];

            tx[11] = (uint8_t)(crc & 0xFF);
            tx[12] = (uint8_t)(crc >> 8);
            int rc = cw_isp_link_xfer(tx, sizeof(tx), rx, sizeof(rx), &rxlen);
            int n = snprintf(out, sizeof(out), "ERASE rc=%d len=%u %02X%02X%02X%02X\n", rc,
                             (unsigned)rxlen, rx[0], rx[1], rx[2], rx[3]);
            reply(out, (size_t)n);
            consume(1);
            continue;
        }

        if (cmd == 'I') {
            /* 用同一组线发 ISP 查询，确认目标是否在线 */
            uint8_t tx[5] = {0x65, 0x01, 0x10, 0, 0};
            uint8_t rx[32] = {0};
            uint16_t rxlen = 0;
            uint16_t crc = isp_crc16(tx, 3);
            char out[96];

            tx[3] = (uint8_t)(crc & 0xFF);
            tx[4] = (uint8_t)(crc >> 8);
            int rc = cw_isp_link_xfer(tx, sizeof(tx), rx, sizeof(rx), &rxlen);
            int n = snprintf(out, sizeof(out), "ISP rc=%d len=%u %02X%02X%02X%02X\n", rc,
                             (unsigned)rxlen, rx[0], rx[1], rx[2], rx[3]);
            reply(out, (size_t)n);
            consume(1);
            continue;
        }

        if (cmd == 'S') {
            /* 借用 CDC 做 SWD 连通性自检：读 DP IDCODE */
            char out[80];
            uint32_t id;

            reply("SWD?\n", 5); /* 先确认链路本身通 */
            id = swd_idcode();
            printf("[cwswd] idcode=0x%08X\n", (unsigned)id);
            int n = snprintf(out, sizeof(out), "SWD %08X%s ack=%d idle=%d pad=%04X\n",
                             (unsigned)id, id ? "" : " NOACK", swd_last_ack(),
                             swd_idle_level(), (unsigned)swd_pad_check());
            reply(out, (size_t)n);
            consume(1);
            continue;
        }

        if (cmd == 'M') {
            /* 双映射扫描：SWDIO 到底是 GPIO4(PA07) 还是 GPIO5(PA08) */
            uint32_t id_a = 0, id_b = 0;
            int ack_a = -1, ack_b = -1;
            char out[80];

            reply("MAP?\n", 5);
            swd_probe_map(&id_a, &ack_a, &id_b, &ack_b);
            int n = snprintf(out, sizeof(out), "MAP A(IO4/CLK5) %08X ack=%d | B(IO5/CLK4) %08X ack=%d\n",
                             (unsigned)id_a, ack_a, (unsigned)id_b, ack_b);
            reply(out, (size_t)n);
            consume(1);
            continue;
        }

        if (cmd == 'Y') {
            /* 用 ISP 写一个最小可运行镜像：SP + 复位向量 + "B ."。
             * 目的：让芯片复位后真的在跑程序，再看 SWD 是否应答
             * （排除"擦空后停在无效状态、调试口被关"这一种可能）。 */
            static const uint8_t ramfun[] = {
                0xF0, 0xB5, 0x03, 0x00, 0x12, 0x4C, 0x01, 0x20, 0x20, 0x60, 0x20, 0x25, 0x98, 0x07,
                0x11, 0xD1, 0x08, 0xE0, 0x67, 0x6A, 0x2F, 0x40, 0xFC, 0xD1, 0x1E, 0x68, 0x86, 0x42,
                0x14, 0xD1, 0x09, 0x1D, 0x1B, 0x1D, 0x12, 0x1F, 0x04, 0x2A, 0x05, 0xD3, 0x08, 0x68,
                0x18, 0x60, 0xF1, 0xE7, 0x49, 0x1C, 0x5B, 0x1C, 0x52, 0x1E, 0x00, 0x2A, 0x09, 0xD0,
                0x08, 0x78, 0x18, 0x70, 0x67, 0x6A, 0x2F, 0x40, 0xFC, 0xD1, 0x1E, 0x78, 0x86, 0x42,
                0xF2, 0xD0, 0x00, 0x20, 0xF0, 0xBD, 0x01, 0x20, 0xF0, 0xBD, 0x00, 0x20, 0x02, 0x40,
                0x38, 0xB5, 0x83, 0x18, 0x5B, 0x1E, 0xEC, 0x24, 0xE4, 0x00, 0xA0, 0x42, 0x02, 0xD3,
                0x16, 0x34, 0xA3, 0x42, 0x09, 0xD3, 0x80, 0x24, 0x64, 0x03, 0xA0, 0x42, 0x0D, 0xD3,
                0xFB, 0x25, 0xED, 0x00, 0x2D, 0x68, 0x2C, 0x19, 0xA3, 0x42, 0x07, 0xD2, 0xFF, 0xF7,
                0xC0, 0xFF, 0x00, 0x28, 0x01, 0xD0, 0x00, 0x20, 0x32, 0xBD, 0x98, 0x20, 0x32, 0xBD,
                0x93, 0x20, 0x32, 0xBD,
            };
            /* 最小镜像：先把 SYSCTRL_CR2.SWDIO 清零（PA07/PA08 恢复 SWD 功能），
             * 然后死循环。ISP BootLoader 为了复用 UART 会把这两位设成 GPIO，
             * Jump 到用户程序时不会复位它，所以必须由用户程序自己打开。 */
            static const uint8_t img[32] = {
                0x00, 0x10, 0x00, 0x20, /* 0x00: SP    = 0x20001000 */
                0x09, 0x00, 0x00, 0x00, /* 0x04: reset = 0x00000009 */
                0x0C, 0x4B,             /* 0x08: ldr r3,[pc,#12] -> 0x18 */
                0x1A, 0x68,             /* 0x0A: ldr r2,[r3]         CR2 */
                0x0C, 0x4C,             /* 0x0C: ldr r4,[pc,#12] -> 0x1C */
                0x22, 0x40,             /* 0x0E: ands r2,r4          clear bit1 + key */
                0x1A, 0x60,             /* 0x10: str r2,[r3] */
                0xFE, 0xE7,             /* 0x12: b . */
                0x00, 0x00, 0x00, 0x00, /* 0x14: pad */
                0x08, 0x40, 0x00, 0x40, /* 0x18: 0x40004008 SYSCTRL_CR2 */
                0xFD, 0xFF, 0x5A, 0x5A, /* 0x1C: 0x5A5AFFFD */
            };
            uint8_t tx[160];
            uint8_t rx[32];
            uint16_t rxlen = 0;
            char out[96];
            size_t n;
            int st_base = -1, st_ram = -1, st_base2 = -1, st_data = -1;

            /* 1) Set BaseAddr = 0x20000800（0x20 后跟两个保留字节再跟地址） */
            static const uint8_t ram_base[6] = {0x00, 0x00, 0x00, 0x08, 0x00, 0x20};
            n = 0;
            tx[n++] = 0x65;
            tx[n++] = 7;
            tx[n++] = 0x20;
            memcpy(tx + n, ram_base, sizeof(ram_base));
            n += sizeof(ram_base);
            { uint16_t c = isp_crc16(tx, n); tx[n++] = (uint8_t)c; tx[n++] = (uint8_t)(c >> 8); }
            if (cw_isp_link_xfer(tx, n, rx, sizeof(rx), &rxlen) == 0) st_base = rx[2];

            /* 2) WriteRamFun 0x27 偏移 0（相对 BaseAddr = 0x20000800） */
            n = 0;
            tx[n++] = 0x65;
            tx[n++] = (uint8_t)(3 + sizeof(ramfun));
            tx[n++] = 0x27;
            tx[n++] = 0x00;
            tx[n++] = 0x00;
            memcpy(tx + n, ramfun, sizeof(ramfun));
            n += sizeof(ramfun);
            { uint16_t c = isp_crc16(tx, n); tx[n++] = (uint8_t)c; tx[n++] = (uint8_t)(c >> 8); }
            if (cw_isp_link_xfer(tx, n, rx, sizeof(rx), &rxlen) == 0) st_ram = rx[2];

            /* 3) Set BaseAddr = 0x00000000（写 Flash 前必须切回） */
            static const uint8_t flash_base[6] = {0};
            n = 0;
            tx[n++] = 0x65;
            tx[n++] = 7;
            tx[n++] = 0x20;
            memcpy(tx + n, flash_base, sizeof(flash_base));
            n += sizeof(flash_base);
            { uint16_t c = isp_crc16(tx, n); tx[n++] = (uint8_t)c; tx[n++] = (uint8_t)(c >> 8); }
            if (cw_isp_link_xfer(tx, n, rx, sizeof(rx), &rxlen) == 0) st_base2 = rx[2];

            /* 4) Write Data 0x28 到 Flash 偏移 0，12 字节 */
            n = 0;
            tx[n++] = 0x65;
            tx[n++] = (uint8_t)(3 + sizeof(img));
            tx[n++] = 0x28;
            tx[n++] = 0x00;
            tx[n++] = 0x00;
            memcpy(tx + n, img, sizeof(img));
            n += sizeof(img);
            { uint16_t c = isp_crc16(tx, n); tx[n++] = (uint8_t)c; tx[n++] = (uint8_t)(c >> 8); }
            if (cw_isp_link_xfer(tx, n, rx, sizeof(rx), &rxlen) == 0) st_data = rx[2];

            int nn = snprintf(out, sizeof(out), "PROG base=%d ramfun=%d base2=%d data=%d\n", st_base,
                              st_ram, st_base2, st_data);
            reply(out, (size_t)nn);
            consume(1);
            continue;
        }

        if (cmd == 'J') {
            /* ISP Jump 0x40 -> 0x00000000：让芯片立刻运行刚写入的程序，
             * 随后立刻用 SWD 探测，验证「芯片在跑用户程序时 SWD 是否应答」。 */
            uint8_t tx[13] = {0x65, 7, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0, 0};
            uint8_t rx[32] = {0};
            uint16_t rxlen = 0;
            uint16_t crc = isp_crc16(tx, 9);
            char out[128];

            tx[9] = (uint8_t)crc;
            tx[10] = (uint8_t)(crc >> 8);
            int rc = cw_isp_link_xfer(tx, 11, rx, sizeof(rx), &rxlen);
            vTaskDelay(pdMS_TO_TICKS(50));

            uint32_t id = swd_idcode();
            int n = snprintf(out, sizeof(out), "JUMP rc=%d len=%u | SWD %08X ack=%d\n", rc,
                             (unsigned)rxlen, (unsigned)id, swd_last_ack());
            reply(out, (size_t)n);
            consume(1);
            continue;
        }

        if (cmd == 'W') {
            /* 采 SWDIO 上的裸位流，判断目标是「完全没应答」还是「应答了但错位」 */
            uint64_t bits = swd_raw_read(40);
            char out[96];
            int n = snprintf(out, sizeof(out), "RAW %010llX\n", (unsigned long long)bits);
            reply(out, (size_t)n);
            consume(1);
            continue;
        }

        if (cmd == 'P') {
            /* GPIO4/GPIO5 自环：两条线从目标上摘下来短接后跑，正常回 LOOP 0A */
            int bits = swd_loopback();
            char out[64];
            int n = snprintf(out, sizeof(out), "LOOP %X\n", bits);
            reply(out, (size_t)n);
            consume(1);
            continue;
        }

        if (cmd == 'R') {
            /* 探测 WriteRamFun 帧长上限：先发 8 字节，再发 64、128 字节 */
            uint8_t tx[160];
            uint8_t rx[32];
            uint16_t rxlen = 0;
            char out[96];
            int r8 = -1, r64 = -1, r128 = -1;
            static const int lens[3] = {8, 64, 128};
            int *res[3] = {&r8, &r64, &r128};

            for (int k = 0; k < 3; k++) {
                size_t n = 0;
                tx[n++] = 0x65;
                tx[n++] = (uint8_t)(3 + lens[k]);
                tx[n++] = 0x27;
                tx[n++] = 0x00;
                tx[n++] = 0x08;
                memset(tx + n, 0, (size_t)lens[k]);
                n += (size_t)lens[k];
                uint16_t c = isp_crc16(tx, n);
                tx[n++] = (uint8_t)c;
                tx[n++] = (uint8_t)(c >> 8);
                if (cw_isp_link_xfer(tx, n, rx, sizeof(rx), &rxlen) == 0) {
                    *res[k] = rx[2];
                }
            }
            int nn = snprintf(out, sizeof(out), "RAMFUN 8=%d 64=%d 128=%d\n", r8, r64, r128);
            reply(out, (size_t)nn);
            consume(1);
            continue;
        }

        if (cmd == 'T') {
            /* 同样的 Write Data 帧写到 RAM(0x20000000)，验证协议本身没问题 */
            uint8_t tx[64];
            uint8_t rx[64];
            uint16_t rxlen = 0;
            char out[200];
            size_t n;
            int st_base = -1, st_w = -1;
            int n1 = 0;

            /* Set BaseAddr = 0x20000000 */
            tx[0] = 0x65;
            tx[1] = 5;
            tx[2] = 0x20;
            tx[3] = 0x00;
            tx[4] = 0x00;
            tx[5] = 0x00;
            tx[6] = 0x20;
            { uint16_t c = isp_crc16(tx, 7); tx[7] = (uint8_t)c; tx[8] = (uint8_t)(c >> 8); }
            if (cw_isp_link_xfer(tx, 9, rx, sizeof(rx), &rxlen) == 0) st_base = rx[2];

            /* Write Data 0x28 @0, 4 bytes */
            n = 0;
            tx[n++] = 0x65;
            tx[n++] = 7;
            tx[n++] = 0x28;
            tx[n++] = 0x00;
            tx[n++] = 0x00;
            tx[n++] = 0xA5;
            tx[n++] = 0x5A;
            tx[n++] = 0x12;
            tx[n++] = 0x34;
            { uint16_t c = isp_crc16(tx, n); tx[n++] = (uint8_t)c; tx[n++] = (uint8_t)(c >> 8); }
            if (cw_isp_link_xfer(tx, n, rx, sizeof(rx), &rxlen) == 0) st_w = rx[2];

            /* Read Data 0x29 @0 cnt16 */
            tx[0] = 0x65;
            tx[1] = 4;
            tx[2] = 0x29;
            tx[3] = 0x00;
            tx[4] = 0x00;
            tx[5] = 16;
            { uint16_t c = isp_crc16(tx, 6); tx[6] = (uint8_t)c; tx[7] = (uint8_t)(c >> 8); }
            int r2 = cw_isp_link_xfer(tx, 8, rx, sizeof(rx), &rxlen);

            n1 = snprintf(out, sizeof(out), "BASE=%d WR=%d\nRD st=%02X:", st_base, st_w, rx[2]);
            for (int i = 0; i < 16 && (i + 3) < rxlen; i++) {
                n1 += snprintf(out + n1, sizeof(out) - (size_t)n1, " %02X", rx[3 + i]);
            }
            n1 += snprintf(out + n1, sizeof(out) - (size_t)n1, "\n");

            reply(out, (size_t)n1);
            consume(1);
            continue;
        }

        if (cmd == 'Z') {
            /* 普通复位（不灌 50kHz）后直接查询：判断空片复位后是否仍在 ISP */
            uint8_t rx[32] = {0};
            uint16_t rxlen = 0;
            char out[96];
            int rc = cw_isp_query_cold(rx, sizeof(rx), &rxlen);
            int n = snprintf(out, sizeof(out), "COLD rc=%d len=%u %02X%02X%02X%02X\n", rc,
                             (unsigned)rxlen, rx[0], rx[1], rx[2], rx[3]);
            reply(out, (size_t)n);
            consume(1);
            continue;
        }

        if (cmd == 'L' || cmd == 'K') {
            /* 读保护等级：L = 只查询 (0x30 0x55)，K = 降级到 Level0 (0x30 0x00)
             * Level2 时 SWD 端口是物理断开的，ISP 仍可降级（会整片擦除）。 */
            uint8_t tx[6] = {0x65, 0x02, 0x30, (cmd == 'L') ? 0x55 : 0x00, 0, 0};
            uint8_t rx[32] = {0};
            uint16_t rxlen = 0;
            uint16_t crc = isp_crc16(tx, 4);
            char out[96];

            tx[4] = (uint8_t)(crc & 0xFF);
            tx[5] = (uint8_t)(crc >> 8);
            int rc = cw_isp_link_xfer(tx, sizeof(tx), rx, sizeof(rx), &rxlen);
            int n = snprintf(out, sizeof(out), cmd == 'L' ? "LEVEL rc=%d len=%u st=%02X lvl=%02X\n"
                                                          : "SETLVL rc=%d len=%u st=%02X lvl=%02X\n",
                             rc, (unsigned)rxlen, rx[2], rx[3]);
            reply(out, (size_t)n);
            consume(1);
            continue;
        }

        if (cmd == 'G') {
            /* 只读采样（不复位）：判断两脚此刻被谁驱动 */
            char out[72];
            int v = swd_pins_sample();
            int pd_io = (v >> 3) & 1, pd_clk = (v >> 2) & 1;
            int pu_io = (v >> 1) & 1, pu_clk = v & 1;
            int n = snprintf(out, sizeof(out), "PINS pd=%d%d pu=%d%d\n", pd_io, pd_clk, pu_io, pu_clk);
            reply(out, (size_t)n);
            consume(1);
            continue;
        }

        if (cmd == 'N') {
            /* 复位后只读采样目标引脚：判断芯片是否还停在 ISP 引导程序 */
            char out[64];
            int v = swd_pins_after_reset();
            int n = snprintf(out, sizeof(out), "NRST pins PA07=%d PA08=%d%s\n",
                             (v >> 1) & 1, v & 1,
                             (v & 1) ? " (PA08 driven high -> still in ISP)" : "");
            reply(out, (size_t)n);
            consume(1);
            continue;
        }

        if (cmd == 'V') {
            /* 目标供电自检：GPIO1 上 1/2 分压 */
            char out[48];
            int n = snprintf(out, sizeof(out), "VDD %d mV%s\n",
                             cw_vdd_target_mv(), cw_vdd_external() ? " EXT" : "");
            reply(out, (size_t)n);
            consume(1);
            continue;
        }

        if (cmd == 'B') {
            if (rx_len < 6) {
                return;
            }
            uint32_t size = le32(rx + 1);
            uint8_t name_len = rx[5];
            if (rx_len < (size_t)6 + name_len) {
                return;
            }
            char name[FW_NAME_LEN];
            memset(name, 0, sizeof(name));
            size_t copy = name_len < (FW_NAME_LEN - 1) ? name_len : (FW_NAME_LEN - 1);
            memcpy(name, rx + 6, copy);
            int rc = fw_store_begin(size, name);
            upload_total = size;
            upload_done = 0;
            consume((size_t)6 + name_len);
            reply(rc == 0 ? "K" : "E", 1);
            if (rc == 0) {
                ui_set_file(name);
                ui_set_status("UPLOAD 0%");
            }
            continue;
        }

        if (cmd == 'D') {
            if (rx_len < 3) {
                return;
            }
            uint16_t len = (uint16_t)(rx[1] | (rx[2] << 8));
            if (rx_len < (size_t)3 + len) {
                return;
            }
            int rc = fw_store_write(rx + 3, len);
            consume((size_t)3 + len);
            reply(rc == 0 ? "K" : "E", 1);
            if (rc == 0 && upload_total > 0) {
                char text[22];
                upload_done += len;
                snprintf(text, sizeof(text), "UPLOAD %lu%%",
                         (unsigned long)(upload_done * 100u / upload_total));
                ui_set_status(text);
            }
            continue;
        }

        if (cmd == 'E') {
            if (rx_len < 9) {
                return;
            }
            uint32_t crc = le32(rx + 1);
            uint32_t auth = le32(rx + 5);
            int rc = fw_store_finish(auth);
            consume(9);
            bool ok = (rc == 0) && (crc == fw_store_crc());
            reply(ok ? "K" : "E", 1);
            ui_set_status(ok ? "FW READY" : "UPLOAD ERR");
            ESP_LOGI(TAG, "upload %s (%u bytes)", ok ? "ok" : "failed",
                     (unsigned)fw_store_size());
            continue;
        }

        /* Unknown byte: drop it and resync. */
        consume(1);
    }
}

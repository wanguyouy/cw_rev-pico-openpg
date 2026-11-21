# 硬件设计方案

目标：把 ESP32-S3 做成一台独立的 CW32 ISP 编程器，支持 ISP / SWD 双模式、
3.3V/5V 软件切换供电、OLED 显示、离线烧录。

## 1. 电源与倒灌防护

```
USB VBUS ──F1(PPTC 500mA)── 5V_RAIL ──U_LDO(AP2112K-3.3)── 3V3 (ESP32 / OLED)
                                  │
                                  ├── SW5V ── D2(SS14) ─┐
                                  │                    ├── VDD_TGT（22µF + 100nF）
                                  └── SW3V3 ───────────┘

GPIO7  ── VDD_EN（低有效，两轨总使能）
GPIO17 ── SEL：0 = 3.3V，1 = 5.0V
VDD_EN 与 SEL 相与后驱动 EN_3V3，再经 74LVC1G04 反相驱动 EN_5V
```

- 单 GPIO 只能二选一，做不到「目标自供电」时两条轨都断开，
  所以总使能单独占一根（GPIO7），电压选择用 GPIO17。
- 3.3V 支路串肖特基 D2，5V 意外时先撞肖特基，不会打进 LDO。
- VDD_TGT 加 10k 泄放电阻，保证断电后能掉到 0V。

开关器件两选一：

| 方案 | 器件 | 说明 |
|---|---|---|
| A（推荐） | TPS22917 ×2 | 自带反向电流阻断、快速输出放电、可调软启，外围最省 |
| B（省成本） | AO3401 ×4（两组背靠背）+ S8050 ×2 | 便宜，但要 4 只 PMOS + 栅极上拉到各自电源轨 |

## 2. 电平转换（支持 5V 目标）

目标端可能是 3.3V 也可能是 5V，信号必须跟着变。用 74LVC1T45（VCCB 可达 5.5V，
带 Ioff，VCCB=0 时输出高阻）。

| 位号 | 器件 | 连接 |
|---|---|---|
| U1 | 74LVC1T45 | VCCA=3V3，A=GPIO4，VCCB=VDD_TGT，B─100Ω→PA07，DIR=GPIO15 |
| U2 | 74LVC1T45 | VCCA=3V3，A=GPIO5，VCCB=VDD_TGT，B─100Ω→PA08，DIR=GPIO16 |

DIR 必须由 GPIO 控制（两根线在两种模式下方向不同）：

| 模式 | U1 (PA07) | U2 (PA08) |
|---|---|---|
| ISP | 高（3.3→5V，TX） | 低（5→3.3V，RX） |
| SWD | 动态切换（SWDIO 双向） | 高（SWCLK 单向输出） |

NRST 走开漏，不经过电平转换，3.3V/5V 都安全：

```
GPIO6 ─4.7k─ Q4(S8050).B    Q4.E=GND    Q4.C ─100Ω→ NRST
                                          └─ 10k → VDD_TGT
```

## 3. 目标电压检测

```
VDD_TGT ──R5 100k──┬── GPIO1 (ADC1_CH0) ──100nF── GND
                   └──R6 100k── GND
```

分压 1/2：3.3V→1.65V，5V→2.5V，都在 ADC 量程内（11 dB 衰减）。

使能供电前先采样：读数 > 0.4V 判定目标板自带电源，不使能并提示。

## 4. 引脚分配

| 功能 | 引脚 |
|---|---|
| SWDIO / UART1_TXD | GPIO4 |
| SWCLK / UART1_RXD | GPIO5 |
| NRST | GPIO6 |
| VDD_EN（或电压选择） | GPIO7 / GPIO17 |
| OLED SCL | GPIO9 |
| OLED SDA | GPIO8 |
| 编码器 A / B / 按键 | GPIO10 / GPIO11 / GPIO12 |
| 电平转换 DIR1 / DIR2 | GPIO15 / GPIO16 |
| ADC 目标电压检测 | GPIO1 |
| 预留 | GPIO2 / 13 / 14 / 17 / 18 |

避开 GPIO0/3/45/46（strapping）与 GPIO19/20（USB）。若模组带八线 PSRAM，
GPIO35~37 不可用。

## 5. BOM

| 位号 | 型号 | 封装 | 说明 |
|---|---|---|---|
| U1,U2 | 74LVC1T45 | SOT-23-6 | 双向电平转换 |
| SW3V3,SW5V | TPS22917 | SOT-23-6 | 负载开关（或 AO3401 ×4） |
| Q4 | S8050 | SOT-23 | NRST 开漏 |
| U_LDO | AP2112K-3.3 | SOT-23-5 | 3.3V / 600mA |
| U_INV | 74LVC1G04 | SOT-23-5 | EN_5V 反相 |
| F1 | PPTC 500mA | 0805 | USB 输入保护 |
| F2 | PPTC 300mA | 0805 | 目标供电保护 |
| D2 | SS14 | SMA | 3.3V 支路反向阻断 |
| D1 | USBLC6-2SC6 | SOT-23-6 | 目标接口 ESD |
| R7,R8,R9 | 100Ω | 0603 | 信号串阻 |
| R2,R13 | 4.7k | 0603 | 基极电阻 |
| R3,R11 | 10k | 0603 | 下拉 / 上拉 |
| R4 | 10k | 0603 | VDD 泄放 |
| R5,R6 | 100k | 0603 | ADC 分压 |
| C1 | 22µF | 0805 | 目标 VDD 储能 |
| C2,C3 | 100nF | 0603 | 去耦 |

## 6. 软件侧待实现

1. ~~`cw_set_vdd(v)`~~ 已实现为 `cwprog/cw_vdd.c`：先查 ADC 判断目标是否自带电源
   → 断电 50 ms → 切 GPIO17 → 上电 30 ms → 强制重进 ISP。
2. 电压/复位来源：CW_Programmer 的连接参数 `F5 <mode> <supply> <reset>`，
   supply 0=3.3V / 1=5.0V / 2=目标自供电，reset 0=SWD / 1=上电 / 2=NRST。
   首次上板时看串口 `[cw] F5 ...` 与 `[cwvdd] tool setting` 两条日志确认编码。
3. 掉电复位时序：断 VDD 前把 GPIO4/5 拉低、电平转换 B 侧置高阻，再断开电源。
4. ~~模式切换：ISP 透传 / SWD~~ 已实现为 `cwprog/swd_dap.c` + `cwprog/dap_hid.c`，
   USB 侧 IF3 为 CMSIS-DAP v1 HID，与 CCID 组成复合设备；DAP 连接时接管
   PA07/PA08，ISP 请求被拒，断开后自动交还 UART1。
5. 离线烧录：16MB Flash 划出 FATFS 分区存 hex，OLED 选文件，
   复用 `0x24→0x27→0x28→0x2A→0x40` 流程并显示进度。

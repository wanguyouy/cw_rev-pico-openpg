# CW Writer 兼容的 ESP32-S3 脱机 ISP 烧录器

把 ESP32-S3 做成一个「CW Writer 编程器」：USB 侧伪装成官方编程器（VID `0x0483` / PID `0x3258`，在 CW_Programmer 中显示为 `CW Writer 0`），UART 侧把收到的 CW32 ISP 帧原样转发给真实芯片。

CW_Programmer V2.2.6 认为自己在操作官方烧录器，实际擦除 / 写入 / 校验 / 跳转全部发生在真实的 CW32L010 上。

本项目基于 [pico-openpgp](https://github.com/polhenarejos/pico-openpgp) 的 SDK，保留了其 USB/CCID 框架，替换掉了原有的 OpenPGP 智能卡逻辑。

## 状态

| 功能 | 状态 |
|---|---|
| USB 连接、芯片识别（Query） | 可用 |
| 在线编程（擦除 / 编程 / 校验 / 读保护查询 / 运行） | 可用 |
| 离线编程（hex 存到 ESP32 后脱机烧录） | 未实现 |
| OpenPGP 智能卡相关功能 | 上游实现仍在，未改动也未验证 |

## 接线

| ESP32-S3 | CW32L010 | 说明 |
|---|---|---|
| GPIO4 | PA07 | SWDIO / UART1_RXD |
| GPIO5 | PA08 | SWCLK / UART1_TXD |
| GPIO6 | NRST | 复位控制 |
| 3V3 | VDD | 目标供电（也可外部供电） |
| GND | GND | 共地 |

ISP 模式下 PA07/PA08 被 BootLoader 复用为 UART1，异步半双工 8N1，初始 115200 bps。

进入 ISP 的时序：`NRST` 拉低 → PA07 输出 50 kHz 方波 → 释放 `NRST` → 保持时钟 20 ms → 停时钟、切换为 UART。

## 构建

本工程使用本地 ESP-IDF（5.5.x 验证过），不再依赖云端编译：

```powershell
. C:\esp\v5.5.5\esp-idf\export.ps1
idf.py -C <本仓库路径> build
idf.py -C <本仓库路径> -p COM7 -b 460800 flash
```

## 使用

1. 接好线，ESP32-S3 通过 USB 连电脑；
2. 打开 CW_Programmer，选择 `CW Writer 0` 并连接；
3. 芯片系列选 `CW32L01`，型号按实际选择（如 `CW32L010F8P6`）；
4. 选择要烧录的 hex，点「在线编程」。

烧录完成后 ESP32 会自动复位目标芯片，程序立即开始运行。

## 代码结构

| 文件 | 作用 |
|---|---|
| `pico-keys-sdk/src/usb/ccid/ccid.c` | CCID 传输层 + CW Writer 命令分发；ISP 帧交给 `cw_isp_prog.c` 转发 |
| `pico-keys-sdk/src/cwprog/cw_isp_prog.c` | UART1 初始化、进入 ISP 时序、帧收发与 CRC 校验、跳转后复位 |

## 协议要点

- ISP 帧格式：`0x65 | len | body | CRC16/X25(lo,hi)`，CRC 覆盖 `0x65`、`len` 和 `body`；
- 芯片按半双工工作，会先回声再应答，接收端需容忍并重建被打坏的帧头；
- 编程器私有命令（`F0` / `F2` / `F4` / `FA` / `FD` 等）统一回 `90 00`，与真实 CW Writer 行为一致；
- `0x2A VerifyData` 的 CRC 由芯片计算，PC 侧逐块比对，无需在 ESP32 内另算。

## 致谢

USB/CCID 与工程骨架来自 [pico-openpgp](https://github.com/polhenarejos/pico-openpgp) 与 [pico-keys-sdk](https://github.com/polhenarejos/pico-keys-sdk)，遵循其原始许可（见 `LICENSE`）。

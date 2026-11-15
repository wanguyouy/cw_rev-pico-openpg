# CW Writer 兼容的 ESP32-S3 脱机 ISP 烧录器

把 ESP32-S3 做成一个「CW Writer 编程器」：USB 侧伪装成官方编程器（VID `0x0483` / PID `0x3258`，在 CW_Programmer 中显示为 `CW Writer 0`），UART 侧把收到的 CW32 ISP 帧原样转发给真实芯片。

CW_Programmer V2.2.6 认为自己在操作官方烧录器，实际擦除 / 写入 / 校验 / 跳转全部发生在真实的 CW32L010 上。另外提供一路 USB CDC 控制台（`COM10`，115200 8N1），可以直接发单字母命令做诊断。

本项目基于 [pico-openpgp](https://github.com/polhenarejos/pico-openpgp) 的 SDK，保留了其 USB/CCID 框架，替换掉了原有的 OpenPGP 智能卡逻辑。

## 状态

| 功能 | 状态 |
|---|---|
| USB 连接、芯片识别（`0x10` Query） | 可用，已在真机验证 |
| 整片擦除（`0x24` ChipErase） | 可用，返回 `0x00` |
| 写 RAM 助手 + 写 Flash（`0x27` / `0x28`） | 可用，`0x28` 已不再回 `0x93` |
| 跳转到用户程序（`0x40` Jump） | 可用，返回 `0x00` |
| 读保护等级查询 / 降级（`0x30`） | 查询可用，实测 `lvl=00` |
| 在线编程（CW_Programmer 全流程） | 可用 |
| 离线编程（hex 存到 ESP32 后脱机烧录） | 已实现，烧录序列与 ISP 写入共用，尚未整机回归 |
| OpenPGP 智能卡相关功能 | 上游实现仍在，未改动也未验证 |

## 接线

| ESP32-S3 | CW32L010 | 说明 |
|---|---|---|
| GPIO4 | PA07 | UART1_RXD（ISP 复用） |
| GPIO5 | PA08 | UART1_TXD（ISP 复用） |
| GPIO6 | NRST | 复位控制（低有效） |
| 3V3 | VDD | 目标供电（也可外部供电） |
| GND | GND | 共地 |

ISP 模式下 PA07/PA08 被 BootLoader 复用为 UART1，异步半双工 8N1，初始 115200 bps。

进入 ISP 的时序：`NRST` 拉低 → GPIO4 输出 50 kHz 方波 → 释放 `NRST` → 保持时钟 20 ms → 停时钟、切换为 UART。

## 构建

本工程使用本地 ESP-IDF（5.5.x 验证过），不再依赖云端编译。命令都在**仓库根目录**执行：

```powershell
& C:\esp\v5.5.5\esp-idf\export.ps1
idf.py build
idf.py -p COM7 flash
```

串口分工：`COM7` 是板载 CH343（烧固件用），`COM10` 是本机 USB CDC（发诊断命令用）。

## 使用

1. 接好线，ESP32-S3 通过 USB 连电脑；
2. 打开 CW_Programmer，选择 `CW Writer 0` 并连接；
3. 芯片系列选 `CW32L01`，型号按实际选择（如 `CW32L010F8P6`）；
4. 选择要烧录的 hex，点「在线编程」。

烧录完成后 ESP32 会自动复位目标芯片，程序立即开始运行。

## CDC 诊断命令

向 `COM10` 发一个 ASCII 字符即可（带 `\n` 也行），回复以 `\n` 结尾：

```powershell
python -c "import serial,time; s=serial.Serial('COM10',115200,timeout=1); time.sleep(0.8); s.write(b'I'); time.sleep(1.5); print(s.read(200))"
```

| 命令 | 作用 | 正常回复 |
|---|---|---|
| `I` | 进 ISP 并发 `0x10` Query | `ISP rc=0 len=25 6500150006…` |
| `X` | 整片擦除 | `ERASE rc=0 len=5 650100E4` |
| `Y` | 写一个 32 字节最小镜像（死循环），验证 `0x28` 写权限 | `PROG base=0 ramfun=0 base2=0 data=0` |
| `J` | ISP `0x40` 跳转到 `0x00000000` | `JUMP rc=0 len=5` |
| `L` / `K` | 读保护等级查询（`0x30`/`0x55`）/ 降级到 Level0 | `LEVEL rc=0 len=7 st=00 lvl=00` |
| `V` | 目标电压 | `VDD 3300 mV` |
| `N` | 复位后只读采样 PA07/PA08 | `NRST pins PA07=1 PA08=0` |
| `G` | 不复位、上下拉采样两个引脚 | `PINS pd=00 pu=11` |
| `Z` | 普通复位后直接 Query（验证复位是否生效） | `COLD rc=-1`（说明没停在 BootLoader） |
| `R` / `T` | `0x27` 帧长上限 / RAM 读写小工具 | `RAMFUN 8=0 64=0 128=0` |
| `Q` | 打印一条被解析的 CWU1 帧 | `CWU1 …` |

## 代码结构

| 文件 | 作用 |
|---|---|
| `pico-keys-sdk/src/usb/ccid/ccid.c` | CCID 传输层 + CW Writer 命令分发；ISP 帧交给 `cw_isp_prog.c` 转发 |
| `pico-keys-sdk/src/cwprog/cw_isp_prog.c` | UART1 初始化、进入 ISP 时序、帧收发与 CRC 校验、跳转后复位 |
| `pico-keys-sdk/src/cwprog/cw_offline.c` | 脱机烧录序列（`0x20` → `0x27` → `0x20` → `0x28` → `0x29`） |
| `pico-keys-sdk/src/cwprog/fw_upload.c` | CDC 控制台与全部诊断命令 |
| `pico-keys-sdk/src/cwprog/fw_store.c` | 把待烧 hex 存进 ESP32 的 flash 分区 |
| `pico-keys-sdk/src/cwprog/cw_vdd.c` | 目标供电/复位方式配置 |
| `pico-keys-sdk/src/ui/` | SSD1306 OLED 状态显示 |
| `tools/fw_upload.py` | PC 侧上传 hex 与对账脚本 |

## 协议要点

- ISP 帧格式：`0x65 | len | body | CRC16/X25(lo,hi)`，CRC 覆盖 `0x65`、`len` 和 `body`；`len` 不含 `0x65` 与 CRC；
- `0x20 SetBaseAddr` 的 body 是 **`0x20` + 2 个保留 `0x00` + 4 字节地址**，即 `len = 7`。少了那两个保留字节，后续写入会一直回 `0x93`（无写权限）；
- `0x27 WriteRamFun` / `0x28 WriteData` 的 `len = 3 + N`（命令 1 + 偏移 2 + 数据 N），偏移是**相对 BaseAddr** 的；
- 写 Flash 的正确顺序：`0x20` 设 base=`0x20000800` → `0x27` 上传 144 字节 RAM 助手 → `0x20` 把 base 切回 `0x00000000` → `0x28` 写数据；
- 芯片按半双工工作，会先回声再应答，接收端需容忍并重建被打坏的帧头；
- 编程器私有命令（`F0` / `F2` / `F4` / `FA` / `FD` 等）统一回 `90 00`，与真实 CW Writer 行为一致；
- `0x2A VerifyData` 的 CRC 由芯片计算，PC 侧逐块比对，无需在 ESP32 内另算。

## 致谢

USB/CCID 与工程骨架来自 [pico-openpgp](https://github.com/polhenarejos/pico-openpgp) 与 [pico-keys-sdk](https://github.com/polhenarejos/pico-keys-sdk)，遵循其原始许可（见 `LICENSE`）。

<div align="center">
  <h1>ESP32-P4-WIFI6-Touch-LCD-4.3</h1>
  <p><strong>由 ESP32-P4 与 ESP32-C6 驱动的 4.3 英寸 480 × 800 MIPI-DSI 触摸开发板</strong></p>
  <p>
    <a href="https://github.com/waveshareteam/ESP32-P4-WIFI6-Touch-LCD-4.3/actions/workflows/esp-idf-examples.yml"><img alt="ESP-IDF 示例构建" src="https://github.com/waveshareteam/ESP32-P4-WIFI6-Touch-LCD-4.3/actions/workflows/esp-idf-examples.yml/badge.svg"></a>
    <a href="LICENSE.txt"><img alt="许可证" src="https://img.shields.io/github/license/waveshareteam/ESP32-P4-WIFI6-Touch-LCD-4.3"></a>
  </p>
  <p>
    <a href="README.md">English</a> ·
    <a href="https://www.waveshare.net/shop/ESP32-P4-WIFI6-Touch-LCD-4.3.htm">🌐 产品页面</a> ·
    <a href="https://docs.waveshare.net/ESP32-P4-WIFI6-Touch-LCD-4.3/">📚 产品文档</a> ·
    <a href="firmware/">📦 出厂固件</a> ·
    <a href="examples/esp-idf/">🧩 ESP-IDF 示例</a> ·
    <a href="examples/arduino/README_ZH.md">🔧 Arduino 示例</a> ·
    <a href="docs/README_ZH.md">📖 仓库指南</a>
  </p>
  <img src="assets/ESP32-P4-WIFI6-Touch-LCD-4.3-details-1.jpg" alt="Waveshare ESP32-P4-WIFI6-Touch-LCD-4.3" width="600">
</div>

---

## ✨ 概述

本仓库提供适用于 Waveshare ESP32-P4-WIFI6-Touch-LCD-4.3 的 ESP-IDF 示例、
对应日期版本的出厂固件源码、出厂恢复镜像和产品原理图。

该开发板集成 ESP32-P4 应用处理器与 ESP32-C6 无线协处理器，并配备竖屏
4.3 英寸电容触摸显示屏、音频输入输出、摄像头和 USB 接口、MicroSD 存储及
40PIN 扩展接口，适用于多媒体界面、边缘应用、智能终端和人机交互项目。

## 🖥️ 硬件概览

| 功能 | 器件 / 接口 |
| --- | --- |
| 主处理器 | ESP32-P4NRW32，双核高性能及单核低功耗 RISC-V CPU |
| 存储 | 封装内 32 MB PSRAM 和外置 32 MB NOR Flash |
| 无线连接 | ESP32-C6-MINI-1，通过 SDIO 通信，提供 2.4 GHz Wi-Fi 6 和 Bluetooth 5 (LE) |
| 显示屏 | 4.3 英寸 480 × 800 IPS LCD，2-lane MIPI-DSI，ST7701 控制器 |
| 触摸 | GT911 电容触摸控制器，最多支持五点触控 |
| 音频 | ES8311 音频编解码器、ES7210 音频前端、双板载麦克风和扬声器接口 |
| 摄像头 | 15PIN 2-lane MIPI-CSI 接口，可选配 OV5647 摄像头 |
| 存储卡 | MicroSD 卡槽，采用四线 SDIO 3.0 |
| USB | USB 2.0 High-Speed OTG 和 USB 转 UART Type-C 接口 |
| 扩展 | 40PIN GPIO 接口，可通过转接方式兼容部分树莓派 HAT |
| 硬件文件 | [产品原理图](schematic/ESP32-P4-WIFI6-Touch-LCD-4.3-schematic.pdf) |

完整规格、接口说明和安全操作方法请参阅
[官方产品文档](https://docs.waveshare.net/ESP32-P4-WIFI6-Touch-LCD-4.3/)。
CI 仅证明源码编译兼容性；硬件引脚和时序还应结合原理图与实际硬件版本验证。

### ESP32-P4 芯片版本配置

ESP-IDF 默认配置使用 `rev3_x`：要求 `CONFIG_ESP32P4_REV_MIN_300`，并使用
250 MHz PSRAM。Rev1.3 仍通过显式的 `config/ci/rev1_3.defaults` 兼容覆盖层支持，
该覆盖层选择 `CONFIG_ESP32P4_REV_MIN_100` 和 200 MHz PSRAM。由
[PR #191](https://github.com/waveshareteam/Waveshare-ESP32-components/pull/191)
发布的 BSP 会将 MIPI PHY 时钟选择交由 ESP-IDF 的芯片版本感知默认值处理；不要在不同配置之间
复制时钟源设置。示例 06–12 选用已发布的 Registry 版本
`waveshare/esp32_p4_wifi6_touch_lcd_4_3==1.0.1`，其中包含 Rev3.x 时钟更新及不使用 INT/RST
的 GT911 地址探测。编译通过不能代替实板屏幕时序验证。

Arduino-ESP32 `3.3.11` 使用独立的预编译核心配置：`ChipVariant=postv3` 对应的库实际为
`CONFIG_ESP32P4_REV_MIN_301` 和 200 MHz PSRAM。因此 Arduino 示例要求 P4 Rev3.1 或更新版本，
不支持 Rev3.0，也不使用 ESP-IDF 的 250 MHz PSRAM 配置。

## 📦 出厂固件

[`firmware/brookesia/`](firmware/brookesia/) 包含对应日期版本的 ESP32-P4 Rev3.x
出厂固件源码快照，使用 ESP-IDF v5.5.5，提供 Brookesia 桌面、ESP-Hosted Wi-Fi、
摄像头、音频、多媒体、设置和小智等应用。本地板级组件保持 ST7701 屏每条数据
lane 500 Mbps 的速率和 30 MHz DPI 时钟，并保留该镜像的 GT911 配置：依次探测
`0x5D`、`0x14` 后轮询，RST 使用 GPIO23，不使用 INT。产品示例使用更新的 Registry
BSP，不驱动 GT911 的 INT 或 RST。

请参阅[出厂固件说明](firmware/README_ZH.md)，按 ESP32-P4 芯片版本下载并烧录对应的合并镜像。

请按照官方产品文档使用正确的烧录工具和恢复流程。复现 Rev3.x 源码请参阅
[默认固件构建说明](firmware/brookesia/README_ZH.md)。编译通过不能替代实际硬件验证。

## 🧪 ESP-IDF 示例

| 示例 | 功能 |
| --- | --- |
| [01_HowToCreateProject](examples/esp-idf/01_HowToCreateProject/) | ESP-IDF 工程结构与组件管理入门 |
| [02_HelloWorld](examples/esp-idf/02_HelloWorld/) | 基础构建、烧录和串口监视流程 |
| [03_i2c_tools](examples/esp-idf/03_i2c_tools/) | I2C 总线检测与设备扫描 |
| [04_wifistation](examples/esp-idf/04_wifistation/) | 通过 ESP32-C6 Hosted 连接接入 Wi-Fi |
| [05_sdmmc](examples/esp-idf/05_sdmmc/) | 通过 SDMMC 访问板载 MicroSD 卡 |
| [06_I2SCodec](examples/esp-idf/06_I2SCodec/) | 通过板级 BSP 使用 ES8311 播放及 ES7210 麦克风回放 |
| [07_Displaycolorbar](examples/esp-idf/07_Displaycolorbar/) | MIPI-DSI 显示初始化和彩条测试 |
| [08_lvgl_demo_v9](examples/esp-idf/08_lvgl_demo_v9/) | LVGL 9 显示与触摸示例 |
| [09_video_lcd_display](examples/esp-idf/09_video_lcd_display/) | MIPI-CSI 摄像头画面显示到 MIPI-DSI 屏幕 |
| [10_mp4_player](examples/esp-idf/10_mp4_player/) | 从 MicroSD 卡播放 MP4 视频 |
| [11_esp_brookesia_phone](examples/esp-idf/11_esp_brookesia_phone/) | ESP-Brookesia 多媒体用户界面 |
| [12_usb_extend_screen](examples/esp-idf/12_usb_extend_screen/) | USB 扩展屏与触摸输入回传 |

请结合各示例内容和官方产品文档确认外设及媒体文件要求。CI 用于验证编译，不能替代
目标开发板和外接配件上的实际运行测试。

## 🛠️ 持续集成

| 开发框架 | 版本 | 默认构建 | 条件配置构建 |
| --- | --- | ---: | ---: |
| ESP-IDF | `v5.5.5` | 12 | 9 |
| ESP-IDF | `v6.0.2` | 12 | 8 |

[ESP-IDF 示例工作流](.github/workflows/esp-idf-examples.yml)会发现
`examples/esp-idf/` 下 12 个直接第一方工程。完整源码运行包含 1 个预检任务和
41 个构建任务：24 个默认配置，以及音频回声、RGB888、Brookesia AI（仅 v5.5.5）
与 USB 最小配置。
仅文档修改只运行预检；出厂固件修改会提示发布审核，但不会作为示例源码处理。未知路径
会触发完整矩阵，差异为空或无法读取时则直接失败。路由与证据边界详见
[CI 指南](docs/CI_ZH.md)。

成功的示例 CI 通道还会上传绑定到对应工程、ESP-IDF 版本和变体的可烧录包。详见
[CI 固件产物](docs/CI_FIRMWARE_ZH.md)。

Arduino 示例使用 Arduino-ESP32 `3.3.11` 与 `ChipVariant=postv3`。该核心内置的 P4
配置要求 Rev3.1 或更新版本，并使用 200 MHz PSRAM，而不是 ESP-IDF Rev3.x 的 250 MHz 配置；在
`examples/arduino/examples/` 下发现 10 个示例。本仓库不声明板载 CAN 或 485 功能。

## 🗂️ 仓库结构

| 路径 | 用途 |
| --- | --- |
| [`examples/esp-idf/`](examples/esp-idf/) | 第一方 ESP-IDF 工程 |
| [`examples/arduino/`](examples/arduino/) | 第一方 Arduino 示例及本板显示适配层 |
| [`firmware/`](firmware/) | 对应日期版本的出厂固件源码及不可变恢复镜像 |
| [`schematic/`](schematic/) | 产品原理图 |
| [`assets/`](assets/) | 文档使用的产品图片 |
| [`config/ci/`](config/ci/) | 仅供 Actions 使用的条件 sdkconfig 覆盖配置 |
| [`docs/`](docs/) | 仓库 CI、组件和硬件维护指南 |
| [`.github/`](.github/) | CI 工作流及示例发现逻辑 |

## 📚 文档

- [中文产品文档](https://docs.waveshare.net/ESP32-P4-WIFI6-Touch-LCD-4.3/)
- [English Product Documentation](https://docs.waveshare.com/ESP32-P4-WIFI6-Touch-LCD-4.3)
- [产品原理图](schematic/ESP32-P4-WIFI6-Touch-LCD-4.3-schematic.pdf)
- [ESP-IDF 示例](examples/esp-idf/)
- [Arduino 示例](examples/arduino/README_ZH.md)
- [仓库指南](docs/README_ZH.md)
- [持续集成](docs/CI_ZH.md)
- [CI 固件产物](docs/CI_FIRMWARE_ZH.md)
- [组件边界](docs/COMPONENTS_ZH.md)
- [硬件审计](docs/HARDWARE_ZH.md)
- [English README](README.md)

## 🤝 支持与贡献

欢迎提交贡献和可复现的问题报告。请提供产品及硬件版本、示例路径、ESP-IDF 版本、
复现步骤、预期行为、实际行为以及相关构建或串口日志。修改本地组件或出厂固件前请先
检查仓库边界。

- [贡献指南](CONTRIBUTING_ZH.md)
- [支持策略](SUPPORT_ZH.md)
- [提交 Issue](https://github.com/waveshareteam/ESP32-P4-WIFI6-Touch-LCD-4.3/issues)
- [微雪技术支持](https://www.waveshare.net/help_center/contact.htm)

## 📄 许可证

本仓库基于 Apache License 2.0 许可。详情请参阅
[LICENSE.txt](LICENSE.txt)。

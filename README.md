<div align="center">
  <h1>ESP32-P4-WIFI6-Touch-LCD-4.3</h1>
  <p><strong>ESP32-P4 and ESP32-C6 powered 4.3-inch 480 × 800 MIPI-DSI touch development board</strong></p>
  <p>
    <a href="https://github.com/waveshareteam/ESP32-P4-WIFI6-Touch-LCD-4.3/actions/workflows/esp-idf-examples.yml"><img alt="ESP-IDF examples" src="https://github.com/waveshareteam/ESP32-P4-WIFI6-Touch-LCD-4.3/actions/workflows/esp-idf-examples.yml/badge.svg"></a>
    <a href="LICENSE.txt"><img alt="License" src="https://img.shields.io/github/license/waveshareteam/ESP32-P4-WIFI6-Touch-LCD-4.3"></a>
  </p>
  <p>
    <a href="README_ZH.md">中文</a> ·
    <a href="https://www.waveshare.com/esp32-p4-wifi6-touch-lcd-4.3.htm">🌐 Product Page</a> ·
    <a href="https://docs.waveshare.com/ESP32-P4-WIFI6-Touch-LCD-4.3">📚 Documentation</a> ·
    <a href="firmware/">📦 Factory Firmware</a> ·
    <a href="examples/esp-idf/">🧩 ESP-IDF Examples</a> ·
    <a href="examples/arduino/README.md">🔧 Arduino Examples</a> ·
    <a href="docs/README.md">📖 Repository Guide</a>
  </p>
  <img src="assets/ESP32-P4-WIFI6-Touch-LCD-4.3-details-1.jpg" alt="Waveshare ESP32-P4-WIFI6-Touch-LCD-4.3" width="600">
</div>

---

## ✨ Overview

This repository provides ESP-IDF examples, source for its dated factory
firmware, factory recovery images, and a schematic for the Waveshare
ESP32-P4-WIFI6-Touch-LCD-4.3.

The board combines an ESP32-P4 application processor with an ESP32-C6 wireless
coprocessor, a portrait 4.3-inch capacitive-touch display, audio input and
output, camera and USB interfaces, microSD storage, and a 40-pin expansion
header. It is designed for multimedia interfaces, edge applications, smart
terminals, and human-machine interaction projects.

## 🖥️ Hardware Overview

| Feature | Device / interface |
| --- | --- |
| Main processor | ESP32-P4NRW32 with dual-core high-performance and single-core low-power RISC-V CPUs |
| Memory | 32 MB in-package PSRAM and 32 MB external NOR flash |
| Wireless | ESP32-C6-MINI-1 over SDIO, providing 2.4 GHz Wi-Fi 6 and Bluetooth 5 (LE) |
| Display | 4.3-inch 480 × 800 IPS LCD, 2-lane MIPI-DSI, ST7701 controller |
| Touch | GT911 capacitive controller with up to five touch points |
| Audio | ES8311 audio codec and ES7210 audio front end with dual onboard microphones and a speaker connector |
| Camera | 15-pin 2-lane MIPI-CSI connector; optional OV5647 camera support |
| Storage | MicroSD card slot using four-line SDIO 3.0 |
| USB | USB 2.0 High-Speed OTG and USB-to-UART Type-C ports |
| Expansion | 40-pin GPIO header compatible with selected Raspberry Pi HATs through an adapter |
| Hardware file | [Product schematic](schematic/ESP32-P4-WIFI6-Touch-LCD-4.3-schematic.pdf) |

For complete specifications, connector details, and safe operating guidance,
see the [official product documentation](https://docs.waveshare.com/ESP32-P4-WIFI6-Touch-LCD-4.3).
CI proves source compatibility only; hardware pin and timing validation should
also use the schematic and the board revision in hand.

### ESP32-P4 silicon profiles

The default ESP-IDF profile is `rev3_x`: it requires `CONFIG_ESP32P4_REV_MIN_300`
and uses 250 MHz PSRAM. Rev1.3 boards remain supported through the explicit
`config/ci/rev1_3.defaults` compatibility overlay, which selects
`CONFIG_ESP32P4_REV_MIN_100` and 200 MHz PSRAM. The BSP released from
[PR #191](https://github.com/waveshareteam/Waveshare-ESP32-components/pull/191)
leaves MIPI PHY clock selection to ESP-IDF's revision-aware default; do not copy
a clock-source setting between profiles. Examples 06–12 select the published
Registry release `waveshare/esp32_p4_wifi6_touch_lcd_4_3==1.0.1`, which includes
the Rev3.x clock update and no-INT/RST GT911 address probing. A successful
compile does not validate panel timing on a physical board.

Arduino-ESP32 `3.3.11` is a separate prebuilt-core profile: its
`ChipVariant=postv3` libraries resolve to `CONFIG_ESP32P4_REV_MIN_301` and
200 MHz PSRAM. Arduino examples therefore require P4 Rev3.1 or newer; they do
not support Rev3.0 and do not use the ESP-IDF 250 MHz PSRAM profile.

## 📦 Factory Firmware

[`firmware/brookesia/`](firmware/brookesia/) contains the source snapshot for
the dated ESP32-P4 Rev3.x factory image built with ESP-IDF v5.5.5. It provides
the Brookesia launcher, ESP-Hosted Wi-Fi, camera, audio, multimedia, settings,
and Xiaozhi applications. Its local board component keeps the ST7701 panel at
500 Mbps per data lane and a 30 MHz DPI clock. It also preserves the image's
GT911 configuration: polling after probing `0x5D` then `0x14`, with reset on
GPIO23 and no interrupt input. The product examples instead use the newer
Registry BSP, which drives neither GT911 INT nor RST.

See the [firmware guide](firmware/README.md) to download and flash the combined image for your ESP32-P4 chip revision.

Follow the official product documentation for the required flashing tool and
recovery procedure. See the [default firmware build guide](firmware/brookesia/README.md)
to build the Rev3.x source. Compilation does not replace hardware validation.

## 🧪 ESP-IDF Examples

| Example | Focus |
| --- | --- |
| [01_HowToCreateProject](examples/esp-idf/01_HowToCreateProject/) | ESP-IDF project structure and component-manager introduction |
| [02_HelloWorld](examples/esp-idf/02_HelloWorld/) | Basic build, flash, and serial-monitor workflow |
| [03_i2c_tools](examples/esp-idf/03_i2c_tools/) | I2C bus inspection and device scanning |
| [04_wifistation](examples/esp-idf/04_wifistation/) | Wi-Fi station through the ESP32-C6 hosted connection |
| [05_sdmmc](examples/esp-idf/05_sdmmc/) | Onboard microSD access over SDMMC |
| [06_I2SCodec](examples/esp-idf/06_I2SCodec/) | ES8311 playback and ES7210 microphone loopback through the board BSP |
| [07_Displaycolorbar](examples/esp-idf/07_Displaycolorbar/) | MIPI-DSI display initialization and color bars |
| [08_lvgl_demo_v9](examples/esp-idf/08_lvgl_demo_v9/) | LVGL 9 display and touch demonstration |
| [09_video_lcd_display](examples/esp-idf/09_video_lcd_display/) | MIPI-CSI camera preview on the MIPI-DSI display |
| [10_mp4_player](examples/esp-idf/10_mp4_player/) | MP4 playback from a microSD card |
| [11_esp_brookesia_phone](examples/esp-idf/11_esp_brookesia_phone/) | ESP-Brookesia multimedia user interface |
| [12_usb_extend_screen](examples/esp-idf/12_usb_extend_screen/) | USB extended display with touch input forwarding |

Review each example and the official product documentation for its peripheral
and media prerequisites. CI validates compilation; it does not replace testing
on the target board and connected accessories.

## 🛠️ Continuous Integration

| Surface | Version | Default builds | Conditional builds |
| --- | --- | ---: | ---: |
| ESP-IDF | `v5.5.5` | 12 | 9 |
| ESP-IDF | `v6.0.2` | 12 | 8 |

The [ESP-IDF examples workflow](.github/workflows/esp-idf-examples.yml) discovers
the 12 direct first-party projects under `examples/esp-idf/`. A full source run
contains one preflight job and 41 build jobs: 24 defaults plus audio-echo,
RGB888, Brookesia-AI (v5.5.5 only), and USB-minimal configurations. Documentation-only changes run the
preflight without ESP-IDF builds; factory-firmware changes are reported for
release review but are never treated as example sources. Unknown paths trigger
the complete matrix, while an empty or unreadable diff fails closed. See the
[CI guide](docs/CI.md) for routing and evidence boundaries.

Successful example-CI lanes also upload a SHA-bound flash package for their
specific project, ESP-IDF version, and variant. See [CI firmware artifacts](docs/CI_FIRMWARE.md).

The Arduino examples use Arduino-ESP32 `3.3.11` with
`ChipVariant=postv3`. That core's bundled P4 configuration is Rev3.1-or-newer
with 200 MHz PSRAM, rather than the ESP-IDF Rev3.x 250 MHz profile. The ten
sketches are discovered below `examples/arduino/examples/`; the repository does
not claim onboard CAN or RS485 functionality.

## 🗂️ Repository Layout

| Path | Purpose |
| --- | --- |
| [`examples/esp-idf/`](examples/esp-idf/) | First-party ESP-IDF projects |
| [`examples/arduino/`](examples/arduino/) | First-party Arduino sketches and board display adapter |
| [`firmware/`](firmware/) | Dated factory-firmware source and immutable recovery images |
| [`schematic/`](schematic/) | Product schematic |
| [`assets/`](assets/) | Product images used by the documentation |
| [`config/ci/`](config/ci/) | Conditional sdkconfig overlays used only by Actions |
| [`docs/`](docs/) | Repository CI, component, and hardware-maintenance guides |
| [`.github/`](.github/) | CI workflow and example discovery logic |

## 📚 Documentation

- [Product Documentation](https://docs.waveshare.com/ESP32-P4-WIFI6-Touch-LCD-4.3)
- [中文产品文档](https://docs.waveshare.net/ESP32-P4-WIFI6-Touch-LCD-4.3/)
- [Product Schematic](schematic/ESP32-P4-WIFI6-Touch-LCD-4.3-schematic.pdf)
- [ESP-IDF Examples](examples/esp-idf/)
- [Arduino Examples](examples/arduino/README.md)
- [Repository Guide](docs/README.md)
- [Continuous Integration](docs/CI.md)
- [CI Firmware Artifacts](docs/CI_FIRMWARE.md)
- [Component Boundaries](docs/COMPONENTS.md)
- [Hardware Audit](docs/HARDWARE.md)
- [中文 README](README_ZH.md)

## 🤝 Support and Contributions

Contributions and reproducible issue reports are welcome. Include the product
and hardware revision, example path, ESP-IDF version, reproduction steps,
expected behavior, actual behavior, and relevant build or serial logs. Review
the repository boundaries before changing local components or factory firmware.

- [Contributing Guide](CONTRIBUTING.md)
- [Support Policy](SUPPORT.md)
- [Open an Issue](https://github.com/waveshareteam/ESP32-P4-WIFI6-Touch-LCD-4.3/issues)
- [Waveshare Support](https://www.waveshare.com/contact_us)

## 📄 License

This repository is licensed under the Apache License 2.0. See
[LICENSE.txt](LICENSE.txt).

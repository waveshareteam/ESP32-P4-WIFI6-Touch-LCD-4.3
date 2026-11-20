# 固件

[English](README.md) | [简体中文](README_ZH.md)

本目录提供 ESP32-P4-WIFI6-Touch-LCD-4.3 的出厂固件。每个 BIN 都是从 `0x0` 烧录的合并镜像，文件名中的日期即版本。

请按 **ESP32-P4 芯片版本**选择固件，芯片版本与 PCB 版本不同。20261106 固件适用于 rev3.2；20260206 固件适用于 rev1.3。

| 文件 | 用途 |
| --- | --- |
| [ESP32-P4-WIFI6-Touch-LCD-4.3_FactoryOnly_20261106.bin](ESP32-P4-WIFI6-Touch-LCD-4.3_FactoryOnly_20261106.bin) | ESP32-P4 rev3.2 出厂演示固件 |
| [ESP32-P4-WIFI6-Touch-LCD-4.3_FactoryOnly_20260206.bin](ESP32-P4-WIFI6-Touch-LCD-4.3_FactoryOnly_20260206.bin) | ESP32-P4 rev1.3 出厂演示固件 |

在本目录执行以下命令，将 `PORT` 替换为开发板对应的串口。擦除 Flash 会清除已保存的设置和用户数据。

```bash
python -m pip install esptool
python -m esptool --chip esp32p4 --port PORT erase-flash
```

然后按芯片版本，**只执行下面对应的一条烧录命令**。

ESP32-P4 rev3.2：

```bash
python -m esptool --chip esp32p4 --port PORT write-flash 0x0 ESP32-P4-WIFI6-Touch-LCD-4.3_FactoryOnly_20261106.bin
```

ESP32-P4 rev1.3：

```bash
python -m esptool --chip esp32p4 --port PORT write-flash 0x0 ESP32-P4-WIFI6-Touch-LCD-4.3_FactoryOnly_20260206.bin
```

相关固件源码见 [`brookesia/`](brookesia/README_ZH.md)。

# Firmware

[English](README.md) | [简体中文](README_ZH.md)

Factory firmware for the ESP32-P4-WIFI6-Touch-LCD-4.3. Each BIN is a merged image flashed at `0x0`; the date in its file name is its version. 

Select the firmware for your **ESP32-P4 chip revision**, which is distinct from the PCB revision. The 20261106 firmware is for rev3.2; the 20260820 and 20260206 firmware images are for rev1.3.

| File | Purpose |
| --- | --- |
| [ESP32-P4-WIFI6-Touch-LCD-4.3_FactoryOnly_20261106.bin](ESP32-P4-WIFI6-Touch-LCD-4.3_FactoryOnly_20261106.bin) | Factory demo for ESP32-P4 rev3.2 |
| [ESP32-P4-WIFI6-Touch-LCD-4.3_FactoryOnly_20260820.bin](ESP32-P4-WIFI6-Touch-LCD-4.3_FactoryOnly_20260820.bin) | Latest factory demo for ESP32-P4 rev1.3 |
| [ESP32-P4-WIFI6-Touch-LCD-4.3_FactoryOnly_20260206.bin](ESP32-P4-WIFI6-Touch-LCD-4.3_FactoryOnly_20260206.bin) | Earlier factory demo for ESP32-P4 rev1.3 |

Run the following commands from this directory, replacing `PORT` with the serial port for your board. Erasing flash removes saved settings and user data.

```bash
python -m pip install esptool
python -m esptool --chip esp32p4 --port PORT erase-flash
```

Then run **one** of the following commands for your chip revision.

ESP32-P4 rev3.2:

```bash
python -m esptool --chip esp32p4 --port PORT write-flash 0x0 ESP32-P4-WIFI6-Touch-LCD-4.3_FactoryOnly_20261106.bin
```

ESP32-P4 rev1.3:

```bash
python -m esptool --chip esp32p4 --port PORT write-flash 0x0 ESP32-P4-WIFI6-Touch-LCD-4.3_FactoryOnly_20260820.bin
```

Related firmware source is in [`brookesia/`](brookesia/).

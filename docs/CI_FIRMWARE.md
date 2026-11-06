# CI firmware artifacts

[中文](CI_FIRMWARE_ZH.md) · [CI guide](CI.md)

A complete ESP-IDF matrix run, such as a manual `all` run or a global build-input
change, creates 42 temporary flashable diagnostic packages. A path-routed run
creates packages only for its selected lanes. Every package comes from that
lane's `flasher_args.json`; none replaces the immutable factory images under
[`firmware/`](../firmware/). Artifacts expire after seven days.

The repository audit locks the checked-in images:

| Image | Size | SHA-256 |
| --- | ---: | --- |
| `ESP32-P4-WIFI6-Touch-LCD-4.3-FactoryOnly-260206.bin` | 33,488,896 bytes | `f87b4b16f49704dc8b05b44953a45c011ca9c244e05547e035b4bfa3db74e022` |
| `ESP32-P4-WIFI6-Touch-LCD-4.3-FactoryOnly-260820.bin` | 16,777,216 bytes | `60f1ea3c77d75c95bd1de75b12642fe11c53873e0290a9b96bb6bb7a4c3aad78` |
| `ESP32-P4-WIFI6-Touch-LCD-4.3-FactoryOnly-261106.bin` | 16,777,216 bytes | `e8cdef6903dc944680fbb3cbdd2165815736be0b11a8129232c1b4e177afd45b` |

Changing any path, size, or digest requires an explicit release review and a
matching audit-policy update; example CI never regenerates these files.

## Provenance and authentication

The shared Python core derives the artifact catalog from this checkout's CI
discovery matrix. It obtains the GitHub repository from `origin`, requires a
clean non-detached branch and its full local HEAD SHA, then inspects only the
newest run for that branch and workflow. The run must be completed successfully
at exactly that HEAD; an explicit run ID is held to the same branch, workflow,
and SHA checks. There is no fallback to an older green run.

Prerequisites are Git, Python 3 (`python` or `py -3` on Windows), and either an
authenticated `gh` login or `GH_TOKEN`/`GITHUB_TOKEN`. Anonymous
artifact download is intentionally unavailable. The tool reports the resolved
repository, branch, HEAD SHA, run ID, and run URL. Before listing or downloading,
it requires one successful preflight, all 42 expected successful build jobs, and
exactly the 42 expected non-empty, non-expired artifacts. It deliberately
rejects a successful partial path-routed run; start or select a complete matrix
run before using the downloader.

## Commands

Windows CMD and PowerShell use the same core:

```text
Flash-CI-Firmware.cmd -PreflightOnly -Artifact firmware-esp-idf-01-howtocreateproject-v5.5.5-default
powershell -File scripts/Flash-CI-Firmware.ps1 -List
powershell -File scripts/Flash-CI-Firmware.ps1 -Port COMx
```

Linux and other POSIX systems use the same policy:

```text
sh Flash-CI-Firmware.sh list
sh Flash-CI-Firmware.sh preflight --artifact firmware-esp-idf-01-howtocreateproject-v5.5.5-default
sh Flash-CI-Firmware.sh flash --artifact firmware-esp-idf-01-howtocreateproject-v5.5.5-default --port /dev/ttyUSB0
```

`list` is live and verifies the complete current-HEAD run before showing all
free-selectable firmware choices. `preflight` downloads one explicitly selected
artifact into ignored `ci-firmware/`, validates it, and never opens a serial port
or starts esptool. `flash` permits exactly one selected artifact per invocation.
It accepts an explicit artifact and port or presents numbered firmware and serial
choices. Optional `pyserial` improves port discovery; an explicit port always
works. The compatibility aliases `-SelfTest` and `-ListOnly` remain offline
catalog/safety checks, while `-PreflightOnly` runs live preflight.
Every preflight/flash validation uses a fresh ignored `ci-firmware/` directory,
so repeated checks cannot reuse an earlier extraction.

## Safety boundary

ZIP paths, symlinks, special entries, duplicate paths, manifests, identities,
hashes, sizes, offsets, overlap, 32 MiB limits, target/chip, baud, and esptool
arguments are validated before flashing. Only manifest-derived offsets and files
are passed to `python -m esptool --chip esp32p4`. Erase operations and overrides
for chip, port, baud, or write mode are rejected. Flashing starts only after the
operator types the exact requested confirmation, and succeeds only when esptool
returns zero and reports `Hash of data verified`.

That proves the programmed bytes, not display, touch, audio, networking, or
other hardware-in-the-loop behavior. Test those functions manually. Do not use
this flow to flash, replace, infer offsets for, or validate checked-in
factory/recovery firmware; those images have separate release and recovery
ownership.

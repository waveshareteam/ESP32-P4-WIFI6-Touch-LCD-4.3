# CI 固件产物

[English](CI_FIRMWARE.md) · [CI 指南](CI_ZH.md)

完整 ESP-IDF 矩阵运行（例如手动选择 `all` 或全局构建输入变更）会为当前 42 个通道各生成
一个临时可烧录诊断包；按路径路由的运行只生成所选通道的包。每个包都来自对应通道的
`flasher_args.json`，绝不替代 [`firmware/`](../firmware/) 中不可变的出厂镜像。产物保留七天
后过期。

仓库审计锁定以下镜像：

| 镜像 | 大小 | SHA-256 |
| --- | ---: | --- |
| `ESP32-P4-WIFI6-Touch-LCD-4.3-FactoryOnly-260206.bin` | 33,488,896 字节 | `f87b4b16f49704dc8b05b44953a45c011ca9c244e05547e035b4bfa3db74e022` |
| `ESP32-P4-WIFI6-Touch-LCD-4.3-FactoryOnly-260820.bin` | 16,777,216 字节 | `60f1ea3c77d75c95bd1de75b12642fe11c53873e0290a9b96bb6bb7a4c3aad78` |
| `ESP32-P4-WIFI6-Touch-LCD-4.3-FactoryOnly-261106.bin` | 16,777,216 字节 | `e8cdef6903dc944680fbb3cbdd2165815736be0b11a8129232c1b4e177afd45b` |

路径、大小或摘要任一变化都必须经过明确的发布审核并同步更新审计策略；示例 CI 不会
重新生成这些文件。

## 来源和认证

共享 Python 核心从当前检出仓库的 CI 发现矩阵推导产物目录。它从 `origin` 解析 GitHub
仓库，要求干净且非 detached 的分支和完整本地 HEAD SHA，然后只检查该分支及工作流最新的
运行。该运行必须在精确 HEAD 上完成并成功；显式提供的运行 ID 也必须匹配相同分支、工作流
和 SHA，不会回退到旧的绿色运行。

前提是 Git、Python 3（Windows 可使用 `python` 或 `py -3`）以及已认证的 `gh` 或
`GH_TOKEN`/`GITHUB_TOKEN`。本工具刻意不支持匿名下载。
它会显示仓库、分支、HEAD SHA、运行 ID 和运行 URL。在列出或下载前，必须确认一个成功预检、
42 个预期构建全部成功，以及恰好 42 个非空、未过期的预期产物。工具会刻意拒绝成功但不完整
的按路径路由运行；使用下载器前应启动或选择一次完整矩阵运行。

## 命令

Windows CMD 和 PowerShell 使用同一核心：

```text
Flash-CI-Firmware.cmd -PreflightOnly -Artifact firmware-esp-idf-01-howtocreateproject-v5.5.5-default
powershell -File scripts/Flash-CI-Firmware.ps1 -List
powershell -File scripts/Flash-CI-Firmware.ps1 -Port COMx
```

Linux 和其他 POSIX 系统同样使用该核心：

```text
sh Flash-CI-Firmware.sh list
sh Flash-CI-Firmware.sh preflight --artifact firmware-esp-idf-01-howtocreateproject-v5.5.5-default
sh Flash-CI-Firmware.sh flash --artifact firmware-esp-idf-01-howtocreateproject-v5.5.5-default --port /dev/ttyUSB0
```

`list` 会在线校验完整的当前 HEAD 运行后，列出可自由选择的固件。`preflight` 将一个明确选择
的产物下载到已忽略的 `ci-firmware/` 并校验，绝不访问串口或启动 esptool。`flash` 每次调用
只允许烧录一个明确选择的产物；可显式提供产物和端口，或从编号的固件与串口列表选择。可选的
`pyserial` 会改进端口发现，显式端口始终可用。兼容别名 `-SelfTest` 和 `-ListOnly` 保持为离线
目录/安全检查；`-PreflightOnly` 执行在线预检。
每一次预检/烧录验证都使用新的已忽略 `ci-firmware/` 目录，因此重复检查不会复用旧解压内容。

## 安全边界

烧录前会验证 ZIP 路径、链接/特殊条目、重复路径、清单、身份、哈希、大小、偏移、重叠、
32 MiB 限制、目标芯片、波特率和 esptool 参数。只会把清单派生的偏移和文件传给
`python -m esptool --chip esp32p4`；擦除操作和对芯片、端口、波特率或写入模式的覆盖都会被
拒绝。只有操作者输入要求的精确确认文本后才会开始烧录，且仅当 esptool 返回零并输出
`Hash of data verified` 时才算成功。

这只能证明字节已写入，不能证明显示、触摸、音频、网络或其他硬件在环行为。请手工测试
这些功能。不得用此流程烧录、替换、推断偏移或验证仓库中的出厂/恢复固件；其发布和恢复
流程具有独立所有权。

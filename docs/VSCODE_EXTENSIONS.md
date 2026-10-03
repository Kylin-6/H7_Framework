# VS Code 插件推荐

H7_Framework 推荐使用 **STM32CubeIDE / CMake Tools 完成构建，EmberProbe 完成烧录、调试和变量观测**。
以下推荐依据维护者本机的 VS Code 插件清单与扩展安装目录整理（2026-10-03）；只选与本项目相关的插件，版本号是本机记录，不是项目锁定要求。不同 VS Code Profile 的启用列表可能不同。

## 优先推荐：EmberProbe

[EmberProbe - MCU Flash & Debug](https://marketplace.visualstudio.com/items?itemName=BakeSheep.emberprobe)
（扩展 ID：`BakeSheep.emberprobe`，本机版本：`0.8.0`）是本项目主要烧录入口。
它将固件下载、断点调试、变量观测和 ELF 分析放在同一个侧边栏，适合 STM32H723 的日常开发。

| 功能 | 在本项目中的用途 |
| --- | --- |
| ELF 烧录与目标检测 | 构建后选择对应板型 ELF，下载到 MC-02 |
| 内置断点调试 | 单步、调用栈、变量与内存检查；无需先配置 Cortex-Debug |
| 实时变量与曲线 | 观察姿态、设备状态与控制目标，导出 CSV 用于实验分析 |
| ELF / 内存分析 | 检查 Flash、DTCMRAM、RAM_DMA 和 RAM_D1 占用 |
| 芯片与故障信息 | 将目标信息、故障寄存器与当前 ELF 符号关联 |
| 可选 Agent Skills | 按需提供烧录校验、变量、外设寄存器和调试操作入口 |

功能及环境要求以 [EmberProbe 官方说明](https://github.com/BakeSheep/EmberProbe-MCU-Flash-Debug) 为准。
本机版本要求 VS Code 1.85+、可用的 OpenOCD；断点调试还需要 ARM GDB。SVD 用于外设寄存器解释，按目标芯片选择；它不是烧录必需项。

### 项目使用流程

1. 用 CMake Tools 选择所需 configure preset，完成配置与构建。
2. 打开 EmberProbe，确认 OpenOCD 环境，选择实际连接的探针和 STM32H723 MCU 目标。
3. 按下面的表选择 ELF，确认后烧录；多个构建目录共存时，核对自动检测结果。
4. 调试与变量观测优先使用带调试信息的 Debug 或三个板型 preset。Release 当前使用 `-g0`，不适合依赖 DWARF 的类型和源码查看。
5. 固件重新构建并烧录后，确认插件使用同一份 ELF，使变量地址、符号与片上固件对应。

| Preset | 选择的 ELF |
| --- | --- |
| Debug | `build/Debug/H7_Framework.elf` |
| Release | `build/Release/H7_Framework.elf` |
| SingleBoard | `build/SingleBoard/H7_Framework.elf` |
| GimbalBoard | `build/GimbalBoard/H7_Framework.elf` |
| ChassisBoard | `build/ChassisBoard/H7_Framework.elf` |

同目录下的 `H7_Framework.map` 用于对应构建的内存分析。旧工作区迁移后应清理 `H7_BSP.elf/.map` 和 `CMakeFiles/H7_BSP.dir`；旧的 `GimbalBoard.elf/.map`、`ChassisBoard.elf/.map` 也不再是当前目标。CMake 不会自动删除改名前的产物，插件自动检测后仍须核对 ELF 路径。
实时变量可从 `Debug_IMU_Data`、应用反馈与设备状态开始；多字段一致读取约束见
[调试 ABI 与调用约束](CODE_GUIDE.md#3-控制状态与通信)。探针采样频率不等于控制周期，曲线不能单独证明 1 kHz 实时性。

EmberProbe、Ozone 和其他调试器使用同一探针时，先结束当前连接再切换工具。
项目保留 [Ozone 工程](../H7_BSP.jdebug) 用于 J-Link 调试；独立旧烧录脚本已移除。

## 构建与源码阅读

| 推荐插件 | 扩展 ID | 推荐理由 |
| --- | --- | --- |
| [STM32CubeIDE for Visual Studio Code](https://marketplace.visualstudio.com/items?itemName=STMicroelectronics.stm32-vscode-extension) | `STMicroelectronics.stm32-vscode-extension` | 本机已有的 STM32 官方集成入口，配套工具包、源码与构建视图 |
| [CMake Tools](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cmake-tools) | `ms-vscode.cmake-tools` | 选择 preset、配置和构建；是本项目 CMake + Ninja 工作流入口 |
| STM32Cube clangd | `STMicroelectronics.stm32cube-ide-clangd` | 本机 ST 配套 C/C++ 导航与诊断；读取编译数据库，保持与当前板型一致 |
| STM32Cube Build Analyzer | `STMicroelectronics.stm32cube-ide-build-analyzer` | 本机 ST 配套 map 分析视图，可检查链接区域占用 |
| [Error Lens](https://marketplace.visualstudio.com/items?itemName=usernamehw.errorlens) | `usernamehw.errorlens` | 就地显示语言服务提供的错误与警告，便于修复编译诊断 |

ST 扩展组已包含多个配套组件，优先从官方集成入口安装并管理，不必逐个安装所有内部组件。
CMake Tools 本身不替代 GNU Arm 编译器、CMake 或 Ninja，环境要求见 [构建说明](../README.md#环境与构建)。
源码跳转不正确时先检查 `.clangd` 指向的 `build/<preset>` 与当前 `compile_commands.json`，不要用另一板型的宏和源码数据库解释当前固件。

## 按需保留的调试工具

| 本机已有插件 | 扩展 ID | 使用场景 |
| --- | --- | --- |
| [Cortex-Debug](https://marketplace.visualstudio.com/items?itemName=marus25.cortex-debug) | `marus25.cortex-debug` | 需要已有 GDB Server/launch.json 工作流时使用；EmberProbe 内置调试不要求它 |
| MemoryView | `mcu-debug.memory-view` | 在兼容调试会话中查看内存 |
| RTOS Views | `mcu-debug.rtos-views` | 在支持的调试适配器中查看任务状态 |
| Peripheral Viewer | `mcu-debug.peripheral-viewer` | 配合兼容适配器与 SVD 查看外设 |
| Memory Inspector | `eclipse-cdt.memory-inspector` | 需要其他调试适配器的内存视图时选择 |
| [Hex Editor](https://marketplace.visualstudio.com/items?itemName=ms-vscode.hexeditor) | `ms-vscode.hexeditor` | 离线查看二进制数据或协议样本 |
| [Hex Hover Converter](https://marketplace.visualstudio.com/items?itemName=maziac.hex-hover-converter) | `maziac.hex-hover-converter` | 阅读 CAN ID、寄存器掩码时快速换算进制 |

Cortex-Debug 本机清单中依赖 Debug Tracker、MemoryView、RTOS Views 和 Peripheral Viewer。
这些视图是否可用取决于当前调试适配器，不因同时安装就自动适配 EmberProbe。

## 日常编辑与文档

| 插件 | 扩展 ID | 使用场景 |
| --- | --- | --- |
| [Git Graph](https://marketplace.visualstudio.com/items?itemName=mhutchie.git-graph) | `mhutchie.git-graph` | 查看提交、分支和差异 |
| [Bookmarks](https://marketplace.visualstudio.com/items?itemName=alefragnani.Bookmarks) | `alefragnani.bookmarks` | 标记控制入口、回调与设备反馈路径 |
| [Markdown All in One](https://marketplace.visualstudio.com/items?itemName=yzhang.markdown-all-in-one) | `yzhang.markdown-all-in-one` | 维护 README 和 docs 文档 |
| [Trailing Spaces](https://marketplace.visualstudio.com/items?itemName=shardulm94.trailing-spaces) | `shardulm94.trailing-spaces` | 查找行尾空格，提交前配合 git diff --check |
| [Code Spell Checker](https://marketplace.visualstudio.com/items?itemName=streetsidesoftware.code-spell-checker) | `streetsidesoftware.code-spell-checker` | 检查英文标识和文档拼写，项目缩写需按实际情况处理 |

本机的主题、图标、截图和 AI 辅助插件可按个人习惯保留，不作为固件构建依赖。
Code Runner 不负责交叉编译本项目，固件构建统一走 CMake preset；C/C++ 格式遵循仓库 `.clang-format`。

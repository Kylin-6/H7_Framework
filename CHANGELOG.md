# 更新日志

本文件记录 H7_BSP 当前阶段的工程进展、已验证结果和仍待完成事项。

格式遵循“日期 + 分类”的方式维护。当前项目尚未形成正式版本号，因此先使用日期条目。

## 2026-09-24

### 已修复

- BMI088 初始化期 SPI 读数被丢弃：`System_Init()` 调用 `BSP_BMI088.Init()` 时，`Is_Initialized()` 尚为 false；原 `SPI2_Callback()` 因此提前返回，未把 DMA 读回的芯片 ID 和配置寄存器值交给驱动，导致初始化校验失败。对照 `rm/demo`，SPI2 配置相同，而 demo 在初始化期会正常分发 SPI 完成回调。
- 移除 `SPI2_Callback()` 的初始化完成检查，继续按 BMI088 加速度计、陀螺仪片选分发；驱动内部仍用 `Init_Finished_Flag` 区分初始化读数和运行期状态处理，EXTI 的初始化保护保持不变。
- 验证：Debug 固件构建和 `git diff --check` 通过；尚未在实机验证初始化与数据输出。

## 2026-09-23

### 本次完成

- 将 `.workbuddy/`、框架可靠性验证方案和框架修复清单调整为本地工作文件：从 Git 索引移除并加入 `.gitignore`，本地实体文件继续保留。
- 在根 README 补充启动三态、设备四态、数据新鲜度、CAN 命令生命周期、发送统计和 Host/实机验证边界。
- 为 System Init、Topic、CAN、各电机与接收设备、BMI088、W25Q64 和 ADC 的关键公共接口补充状态语义、失败条件及调用约束注释；未改变公共 API 或运行行为。
- 运行 Fuzzy、Boundary、Trajectory、FilterPolynomial、CAN、Topic、S.BUS 共 7 个 Host 测试工程，19 项测试全部通过；`H7_BSP` Debug 固件完整构建链接通过。

### 已知问题

- 尚未启用独立 IWDG，也没有记录并上报 RCC 复位原因；任务或中断卡死时缺少独立硬件兜底和复位诊断证据。
- Daemon 只负责超时、状态跃迁和可选离线回调。除达妙电机在掉线跃迁时单次尝试使能外，尚无统一的遥控器失联、关键设备掉线到安全输出的 Application 联动策略。
- `System_Init_GetFailureMask()` 和 `BSP_CAN_GetTxStats()` 已提供诊断数据，但尚未接入独立遥测/诊断模块；`StatusTask` 当前只统一调度 Daemon 检查。
- Tests 尚无根级一键入口和 CI。S.BUS 测试仍使用标准 `assert`，必须使用未定义 `NDEBUG` 的 Debug 配置，否则断言会被编译器移除。
- 尚未建立 CAN 静态带宽预算和场景帧回放，也未完成 FDCAN 回环、关键设备拔线、跌压重启及至少 2 小时长跑验证。
- Debug 构建仍报告 PID 初始化结构体的 C-linkage 警告，以及若干 `volatile` 对象自增弃用警告。当前 DTCMRAM 使用 107568 B / 128 KiB（82.07%），后续增加任务栈和静态对象前需复核水位。

### 后续计划

1. **安全兜底**：启用 IWDG，记录 RCC 复位原因，并在 Application 中为遥控器和关键设备建立有明确时限的安全联动。
2. **遥测可观测**：将启动失败位图、CAN 发送统计、Daemon 跃迁、任务栈水位和 CPU 运行统计接入独立诊断通道。
3. **自动回归**：增加 Tests 根级 CTest/脚本和 CI；把 S.BUS 的标准 `assert` 迁移到不会被 Release 禁用的 CHECK 机制，补齐 EventQueue、协议回放和总线负载测试。
4. **实机验证**：依次完成 FDCAN 回环、设备拔线、上电仲裁、跌压重启和长跑测试，保存可复查的遥测记录。
5. **工程治理**：清理现有编译警告，建立 DTCMRAM、任务栈和 CAN 带宽预算，避免资源增长失去约束。
6. **板级配置层**：参考跃鹿 `robot_def.h` 的配置契约，采用机器人独立目录与构建期选择，逐步收敛 Init、FreeRTOS、Pulse 表和遥测代码中分散的板型 `#if`。
7. **回调与 Pulse 表注册制**：让 EXTI、SPI、TIM 对齐 CAN 的实例/键注册模式，由模块在初始化阶段声明回调和周期服务，减少集中分发表的条件编译。
8. **输入源抽象**：参考 `cmd/standard_cmd` 模式，为 S.BUS、板间帧和视觉输入建立统一 Provider/适配接口，使 `Control_Task` 只消费固定的标准输入槽。
9. **板间协议共享**：参考 `can_comm` 通用模块，将 `0x065`、`0x070`、`0x075` 的 ID、字段布局、缩放和通道映射集中到通信两端共享的协议定义中。

## 2026-06-01

### 已修复

- 修复 FDCAN2 Message RAM 重叠（重要）：CubeMX 生成的 `fdcan.c` 中 `hfdcan2.Init.MessageRAMOffset` 原为 `0`，与 FDCAN1 的 Message RAM 区段完全重叠（FDCAN1/FDCAN2 在 STM32H723 上共享同一块 Message RAM）。双路同时收发会互相覆盖 Filter Table / RxFIFO / TxFIFO，行为未定义。现按三等分布局改为 `853`（FDCAN1=0、FDCAN2=853、FDCAN3=1706），三段均匀且互不重叠。
- `BSP_CAN_SendMsg()` 增加 `len == 0 || data == NULL || hfdcan == NULL` 入参保护：原本仅检查 `len > FDCAN_MAX_PAYLOAD`，未拦截 `len == 0`，导致 `CanTxTask` 启动后会每 1ms 向三路总线发送 DLC=0、ID=0x000 的空帧。
- `CanTxTask` 的 `vTaskDelayUntil` 周期失效：`xLastWakeTime` 原在 `for` 循环体内声明并每次 `xTaskGetTickCount()` 重置，退化为 `vTaskDelay(1) + 执行时间`，失去固定周期补偿。已将 `xLastWakeTime` 移到循环外初始化一次。

### 新增

- 新增 UART BSP 抽象（`User_File/Middleware/BSP/UART/bsp_uart.cpp/.h`），仿 SCUT-Robotlab / 达妙 `drv_uart` 双缓冲范式改写并适配本工程：
  - 接收采用 `HAL_UARTEx_ReceiveToIdle_DMA` + IDLE 中断 + 双缓冲（`Rx_Buffer_0/1` 交替），收不定长帧；切缓冲后记录 `Rx_Timestamp`。
  - 仅接管具备 RX DMA 的 7 路：USART1/2/3、UART5、USART6、UART7、USART10。UART4/8/9 因 STM32H7 DMA1+DMA2 共 16 条 stream 已被占满（7 路 UART 收发 + SPI + ADC），分不到 DMA，暂不接管。
  - UART5 仅有 RX DMA、无 TX DMA，`UART_Transmit_Data()` 检测 `huart->hdmatx == nullptr` 时自动回退阻塞发送。
  - 管理对象（含双 512B DMA 缓冲）全部加 `__attribute__((section(".dma_buffer"), aligned(32)))`，落入 RAM_D1（DMA 可访问、MPU non-cacheable）。
  - 相比模板：抽出 `UART_Get_Manage_Object()` 辅助函数消除 10 路重复 if-else；HAL 回调（`HAL_UARTEx_RxEventCallback` / `HAL_UART_ErrorCallback`）显式用 `extern "C"` 以正确覆写 HAL 弱符号（对齐 `bsp_spi` 约定）。
  - 回调注册模型为「实例分支 + 单回调」（与 `bsp_spi` 同构，区别于 `bsp_can` 的「CAN ID 查表」注册表模型）：`UART_Init(huart, callback)` 把 `void(uint8_t*,uint16_t)` 回调绑定到对应管理对象；回调可传 `nullptr` 退化为轮询模式（DMA 照常收、不触发回调）。
  - 已在根 `CMakeLists.txt` 注册 source 与 include 目录，`Debug` 构建链接通过（RAM_D1 占用约 13.7 KB / 320 KB）。

### 备注

- 上述三项 CAN bug 与 UART 封装的 CubeMX 侧 RX DMA 改 NORMAL 由作者完成，本条目记录最终结论与设计要点。
- **已于 2026-09-23 核对**：FDCAN1/2/3 的 `AutoRetransmission` 当前均为 `DISABLE`，不存在三路配置不一致；是否调整重传策略应在完成总线负载与故障注入验证后另行评估。
- 尚未接入：各路 UART 的用户级回调与 `Init.cpp` 中的 `UART_Init` 绑定（取决于实际外接设备：DBUS 遥控器 / 裁判系统等）。

## 2026-05-16

### 新增

- 新增项目 README，集中说明工程定位、目录结构、启动链路、已实现功能、未实现功能、构建方式和后续建议。
- 新增本更新日志，用于持续记录 BSP 开发进度。
- 新增 `User_File/Task/TransportTask.cpp` 的实际任务入口，原空文件已补齐为可构建骨架。
- 在 `TransportTask` 中接入 `MX_USB_DEVICE_Init()`，USB Device 初始化从 CubeMX 弱任务实现迁移到用户任务实现中。
- 在 `User_File/Task/user_task.h` 中声明 `Ins_Task()` 与 `Transport_Task()`，使用户任务入口与 FreeRTOS 创建逻辑保持一致。
- 在根 `CMakeLists.txt` 中注册 `TransportTask.cpp`，任务文件已纳入构建。
- 新增 `User_File/Task/CanTxTask.cpp` 与 `User_File/Task/StatusTask.cpp`，接管 CubeMX 已创建的 `CanTxTask` 与 `StatusTask` 任务入口。
- 在 `User_File/Task/user_task.h` 中补充 `can_tx_task()` 与 `status_task()` 声明，并在根 `CMakeLists.txt` 注册新增任务文件。

### 已完成

- 完成一轮最小 C/C++ 边界收口：`System_Init()`、HAL GPIO/TIM/SPI 回调、`SPI2_Callback()`、任务入口均显式使用 C ABI。
- 净化 `Init.h`、`callback.h`、`user_task.h` 的 C 可见区域，C++ 依赖移入实现文件。
- README 增补 C/C++ 协作设计、Ozone 调试镜像设计、CMake Tools 解析提示说明。
- README 增补 FreeRTOS 后续任务拆分原则，明确 `InsTask` 只作为姿态数据生产者，云台/底盘/发射通过 `INS_State` 快照和事件通知解耦。
- README 增补通信任务分层原则，区分 CAN 控制链路、USB/串口遥测、遥控/视觉/裁判系统解析与 `TransportTask` 职责边界。
- 将 `Core/Src/freertos.c` 中的 `status_task()` 默认实现改为弱符号，避免用户层 `StatusTask.cpp` 强实现产生重复定义。
- 将 CAN BSP 发送互斥锁迁移到 CMSIS-RTOS V2 API，并在 `MX_FREERTOS_Init()` 任务创建前调用 `BSP_CAN_ConfigInit()` 完成 FDCAN 启动与锁初始化。
- 将 `cmake/stm32cubemx/CMakeLists.txt` 从混合换行规范化为 CRLF，并清理 `MX_LINK_LIBS` 段尾随空白，以规避 VS Code/CMake Tools 自动分析器误报。
- 验证新增用户源文件应手动注册到根 `CMakeLists.txt` 的 `target_sources`；临时测试源文件已从构建列表清理。
- 调整 `.vscode/settings.json`：保留 `cube-cmake`，显式绑定 Debug configure/build preset，移除会触发 unused warning 的 `-DCMAKE_COMMAND=cube-cmake`。
- 修复 SPI5 全双工完成回调的 Tx 长度参数，避免把 `Rx_Buffer_Length` 同时传给 Tx/Rx 长度。
- 清理 `bsp_spi.cpp` 中的尾随空白，使相关文件不再触发 `git diff --check` 的该项报告。
- 将全局 `init_finished` 改为 `volatile bool`，降低初始化完成标志在中断上下文读取时的优化风险。
- 确认 `CMakePresets.json` 提供 `Debug` 与 `Release` preset，使用 Ninja 和 `cmake/gcc-arm-none-eabi.cmake` 工具链。
- 确认根 `CMakeLists.txt` 已接入用户算法、BSP、System、Device、Task 和 SystemView 源码。
- 确认 FreeRTOS 原始 `port.c` 已被过滤，改用 `User_Config/FreeRTOS_Patch/port_patched.c`。
- 确认 `System_Init()` 已接入 SystemView、时间戳、EXTI 优先级、SPI2、SPI6、TIM5 和 BMI088 初始化。
- 确认 `callback.cpp` 已接管 HAL EXTI、TIM、SPI2 回调分发。
- 确认 `Core/Src/main.c` 中的 `HAL_TIM_PeriodElapsedCallback()` 是弱定义，用户层 `callback.cpp` 可以覆盖。
- 确认 `InsTask` 由任务通知驱动，收到 BMI088 陀螺仪数据完成通知后执行 `BSP_BMI088.EKF_Calculate()`。
- 确认 BMI088 已建立 EXTI 数据就绪、SPI DMA 读取、SPI 完成回调、任务通知、EKF 解算的主链路。
- 确认 SPI/ADC 管理对象已放入 `.dma_buffer`，链接脚本将该段放入 RAM_D1。
- 确认 RAM_D1 MPU 配置为 non-cacheable，降低 H7 D-Cache 与 DMA 一致性风险。
- 确认 `configCHECK_FOR_STACK_OVERFLOW` 已启用，`vApplicationStackOverflowHook()` 已实现。
- 确认主要源码目录中已无空文件。

### 构建验证

- 构建命令：`cmake --build build/Debug`
- 构建结果：成功
- 输出文件：`build/Debug/H7_BSP.elf`
- ELF 大小：3254876 B
- 最近构建时间：2026-05-16 00:52:35

内存占用摘要：

| 区域 | 使用量 | 总量 | 使用率 |
| --- | ---: | ---: | ---: |
| DTCMRAM | 103968 B | 128 KB | 79.32% |
| RAM_D1 | 6336 B | 320 KB | 1.93% |
| RAM_D2 | 0 B | 32 KB | 0.00% |
| RAM_D3 | 0 B | 16 KB | 0.00% |
| ITCMRAM | 0 B | 64 KB | 0.00% |
| FLASH | 150608 B | 1024 KB | 14.36% |

### 仍未完成

- `TransportTask` 仍只有 USB Device 初始化和 `osDelay(1)` 循环，尚无实际通信协议、收发队列、遥测输出或命令解析。
- ~~CAN/FDCAN 用户层 BSP 尚未迁移~~ → **已于 2026-06-01 完成**：`bsp_can` v2 双通道架构（周期 + 异步）实现并修复三处 Bug。
- Power/ADC 模块已有代码，但 `System_Init()` 尚未调用 `ADC_Init()` 与 `BSP_Power.Init()`。
- BMI088 加热器默认关闭，温度读取与 128 ms PID 温控周期尚未接入当前调度链路。
- `Task1s_Callback()`、TIM7 1 ms 分支等周期任务仍为空或注释状态。
- ISR 与任务之间共享的 BMI088 内部 `Init_Finished_Flag` 及 ready/update/transfering 标志仍是普通 `bool`，后续需明确并发语义。
- EKF 矩阵表达式在 Debug / `-O0` 下可能有较大栈压力，需要继续观察 `InsTask` 栈水位。
- 工作区存在大量未提交/未跟踪改动，需要后续按主题整理提交。

### 风险与注意

- DTCMRAM 当前使用率约 79.32%，FreeRTOS heap、`.data`、`.bss`、栈都在该区域，需要关注后续任务和全局对象增长。
- SPI6 当前走阻塞传输，这是因为 BDMA 可访问内存区域限制尚未单独处理。
- 当前 BMI088 读取策略已经避免在 SPI 完成回调中继续发起新的 DMA 传输，后续修改时应保持这一原则，避免 DMA-in-DMA 竞态复发。
- 若后续启用 BMI088 加热器，需要先确保 ADC 电压采样和电源组件初始化有效，否则加热功率计算没有可靠输入。

## 2026-05-18

### 新增

- 新增 `.clang-format`，设置 `ColumnLimit: 0`，关闭自动换行，保持长行代码可读性。
- 在 `bsp_can.c` 中规划新增 `Tx_Msg_Buffer[3]` 静态周期帧缓冲区，每路 CAN 对应一个槽位。
- 规划 `BSP_CAN_Init_Msg()` 函数，用于初始化三路 CAN 的默认发送帧（ID/len/data 清零）。
- 规划 `BSP_CAN_SendPer()` 函数，遍历 `Tx_Msg_Buffer[3]` 统一发送三路周期帧，使用 `&=` 汇总成功状态。
- 规划 `CanTxTask` 任务主循环，以 `vTaskDelayUntil` 实现精确 1ms 周期，结合异步队列处理插队消息。

### 问题发现

- **FDCAN2 配置缺失（重要）**：CubeMX 生成的 `fdcan.c` 中 FDCAN2 的 `TxFifoQueueElmtsNbr` 和 `RxFifo0ElmtsNbr` 均为 0，且未配置 NVIC 中断。通过 `BSP_CAN_SendMsg(&hfdcan2, ...)` 发送将永远返回 `false`。需在 CubeMX 中重新配置 FDCAN2，分配 TX FIFO（8 槽）、RX FIFO0（16 槽）及过滤器，重新生成代码。
  - **更新（2026-06-01 已解决）**：`fdcan.c` 现已配置 FDCAN2 `RxFifo0ElmtsNbr = 16`、`TxFifoQueueElmtsNbr = 8`，并在 `HAL_FDCAN_MspInit` 中配置了 `FDCAN2_IT0/IT1` 的 NVIC。随后发现并修复了由此暴露的 Message RAM 重叠问题（见 2026-06-01 条目）。

### 仍未完成

- `BSP_CAN_Init_Msg()`、`BSP_CAN_SendPer()`、`CanTxTask` 周期发送 + 异步队列逻辑均已写入实现（见 `bsp_can.c` / `CanTxTask.cpp`），但 `SendMsg` 的 `len == 0` 保护与 `vTaskDelayUntil` 周期写法仍待收口（见 2026-06-01 条目）。

## 2026-04-10 之前

### 已有基础

- CubeMX 工程已生成 STM32H723ZG 外设初始化代码。
- 工程已具备 CMake + Ninja 构建结构。
- `User_File/Middleware/Algorithm/` 已包含基础数学、矩阵、四元数、PID、FSM、队列、斜坡、Kalman、EKF、频率滤波等算法组件。
- `User_File/Middleware/BSP/` 已有 SPI 与 ADC 抽象雏形。
- `User_File/Device/Components/` 已有 BMI088 与 Power 组件。
- SystemView、SEGGER RTT、USB Device、FreeRTOS、CMSIS-DSP 等依赖已放入工程。

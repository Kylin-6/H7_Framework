# 更新日志

本文件记录 H7_BSP 当前阶段的工程进展、已验证结果和仍待完成事项。

格式遵循“日期 + 分类”的方式维护。当前项目尚未形成正式版本号，因此先使用日期条目。
旧日期条目记录当时的构建和测试快照，其中的数量及“尚未完成”不表示当前状态；
当前能力以根 README 和对应模块文档为准。

## 2026-10-04

### 合入上游算法与 ADC 修复

- 从 H7_BSP main `300c220` 选择性移植 `9fc7258`、`c172e36`、`300c220` 的 Matrix、Kalman 与 EKF 修复。求逆使用缩放部分选主元并检查非有限值；滤波测量更新返回 bool，失败时保留当前 X/P 并清零 K；EKF 分阶段校验并复用中间结果。
- 从 `38266f4` 仅移植 ADC 采样长度上界检查，在 HAL 调用前拒绝超过缓冲容量的请求。同步公开接口注释与使用文档；现有任务、设备安全与应用构建选择保持原有实现。
- SingleBoard、GimbalBoard、ChassisBoard 配置与构建通过；GNU Arm 实际固件参数下模板实例化通过，包含非方形观测维度。通过 stdin 执行实际算法头文件及 ADC 实现的边界验证：矩阵尺度、换行、奇异与非有限值，滤波正常修正、失败状态保护与恢复，ADC 长度边界与 HAL 失败传播；地址与未定义行为检查通过，沙箱下未启用泄漏检测。未新增验证源码或桩文件，文档链接及 git diff --check 通过；未完成实机数值与采样验证。

### 设备安全职责与 App 流程

- Chassis 与 Shoot 同样改为构建期源码选择：关闭时不编译、不调度、不发布本地应用反馈，内部移除功能条件编译。任务向 RobotCmd_Init 显式传入 Shoot 是否编入，未编入时拒绝射击事件；云台板的远端底盘命令与反馈、静态消息通道及共享设备服务保持原有契约。同步底盘替换例程与模块接口说明。
- 云台启用改为构建期源码选择：关闭时不编译、不调度、不发布云台反馈；INS 独立发布。Gimbal 与两套单轴例程内部移除功能条件编译，仅在单板任务边界保留 `#if GIMBAL`，不新增替代反馈文件或空实现。
- 保留 Daemon + Device + App 分层：Daemon 只判断反馈活性和状态跃迁；DJI、DM 驱动自行保护离线输出，StatusTask 在 CheckAll 后调用设备 ServiceAll，不依赖 App 持续调用控制接口。
- DJI 独立清零未就绪成员的输出与积分并覆盖共享周期帧，保留其他成员槽位。DM 离线回调登记安全动作，由设备服务覆盖协议安全目标、提交 Disable 并补交失败项；不自动 ClearError。
- Gimbal 移除生产实现的 PrepareControl 设备准入流程，按初始化、功能模式和 INS 有效性表达许可。UpdateTarget 合并未就绪、首次就绪／恢复及进入 LOCK 的捕获条件；保留未就绪每周期捕获，以及恢复后 IMU 等待新序号目标的行为。
- Shoot 模式分支计算目标和事件保持状态，随后统一选择拨弹外环并调用一次 Control；RobotCmd 复用失联和切源时的射击事件清理，保持发布频率和仲裁顺序。Chassis 未新增跟随或自旋行为。
- 移除 QD4310 驱动支持及构建集成，删除本轮新增的 MotorSafety 主机测试与对应协作要求；WS2812 保持现有灯效，不新增状态显示规则或掉线检测。
- 同步模块代码示例、总览、快速上手和开发入口；达妙与 GM6020 单轴云台移植例程也改为直接表达功能许可，由 UpdateTarget 合并捕获条件，移除旧 PrepareControl 流程。历史条目继续保留当时语境。

### 验证

- 底盘/发射源码选择改动后，单板全部 OFF、仅 Chassis ON、仅 Shoot ON、全部 ON（后三者使用独立目录）及 GimbalBoard、ChassisBoard 六种配置构建通过。编译清单与 ELF 确认关闭时排除对应 App，云台板保留远端底盘收发，共享 DJI 服务保留；开启分支代码与此前一致。生产函数文档示例与源码匹配，四麦轮完整例程语法检查及文档链接、git diff --check 通过；未新增测试文件。
- 云台源码选择改动后，单板云台 OFF、单板云台 ON（独立构建目录）、GimbalBoard、ChassisBoard 均构建通过；编译清单与符号检查确认关闭时排除云台，开启分支源码与此前一致。两套单轴文档例程编译检查通过。
- 生产代码改动后 SingleBoard、GimbalBoard、ChassisBoard 配置与构建通过，逐路径核对捕获、目标和事件流程；文档示例与生产函数一致，git diff --check 通过。云台板仍有既有 PID typedef 编译警告。
- 未完成实机 CAN 断线、协议失能、姿态恢复和电机闭环验证；构建通过不代表设备已经执行安全输出。

## 2026-10-03

### 构建名称与开发入口

- Debug、Release、SingleBoard、GimbalBoard、ChassisBoard 均生成 `H7_Framework.elf` 和同名 `.map`，以 `build/<preset>/` 区分配置。旧 `H7_BSP` 构建目标停用；下方历史条目中的旧产物名保留当时语境。
- 烧录入口改为 EmberProbe；迁移工作区时清理旧名称产物并重新选择 ELF。保留 Ozone 调试工程。

## 2026-09-28

### 当前代码已实现

- CMake 增加 SingleBoard、GimbalBoard、ChassisBoard 构建目标，分别选择应用、设备、任务源码；BoardConfig 管硬件资源，TransportConfig 管固定跨板映射。SingleBoard 硬件应用默认关闭。
- RobotCmd 使用无堆分配的 `Output<T>`，通过 `IsBound()` 拒绝未绑定的初始化；`LocalPublisher` 发布到本地 Topic，`RemotePublisher` 将底盘命令交给固定 Transport。
- `ChassisCmd` 由云台板通过标准 CAN ID `0x141` 发往底盘板；`ChassisFeedback` 由底盘板通过 `0x222` 返回。两方向按实际 RX 时间、序号与 100 ms 时效检查后进入本地 Message Center；协议为 latest-value，无会话标识或 ACK。
- 三路 FDCAN 离散命令各有静态 FIFO 和 pending 帧；同总线保持顺序，单总线失败不阻断其他总线。`CAN_Tx_Perform()` 保留最新值周期槽语义。
- DM 离线首次使能入队失败时记录 `recover_pending`；`StatusTask` 调用 `ServiceAll()` 至少间隔 50 ms 重试入队。Gimbal 两轴关闭此机制，使用自己的恢复状态机。
- Chassis、Shoot 和 DJI 控制链统一 rad/rad/s；直驱 M3508 摩擦轮显式配置 `gear_ratio = 1.0`，默认目标 `25 rad/s`。

### 文档同步

- 以当前源码修订架构、任务、双板、设备和协议契约；将 2026-09-25 框架审查标为历史快照。
- 未修改生产行为。Referee/DBUS 解析、输入仲裁、完整发射状态机等仍为待完成项，实机验证尚未完成。

## 2026-09-27

### 云台改用双达妙电机

- 云台移除 QD4310 依赖，复用 Class_DMMotor；独立 QD4310 驱动及测试保留。继续由 ControlTask 以 1 kHz 调度，反馈为 100 Hz。
- Yaw 使用 INS 最短路径角误差与速度串级 PID，输出 MIT 纯转矩；Pitch 参考 Meta 将姿态误差转换为 MIT 电机位置/速度目标。配置集中到 Gimbal_Config.h，角度、速度和转矩统一为 rad、rad/s、N·m。
- 默认关闭云台编译选项。Yaw 转矩环增益为零待整定；协议量程、Pitch MIT 增益和机械限位来自参考示例，不能当作本机标定值。原 QD 电流环增益和绝对角限位不再沿用。
- 初始化改为非阻塞注册；使能每 20 ms 重试、两秒超时、一秒退避。故障或 DISABLED 覆盖零 MIT 输出并重试失能；反馈持续有效 100 ms 后自动恢复，重置 PID 并捕获当前姿态。IMU 模式恢复后仅接受新发布的目标，LOCK 保持捕获姿态。
- 达妙新增反馈一致快照和按实例关闭离线自动使能的接口；SetMIT/SetTorque 返回软件周期槽提交结果。云台独占自身恢复决策，不自动清错、置零或修改电机持久化模式。
- 保持 GimbalCmd/GimbalFeedback 字段布局，说明速度前馈及 enabled 的反馈语义；更新应用、达妙驱动及测试文档和参考许可。

### 验证

- 默认 Debug、仅云台 Debug、三应用全开 Debug/Release 均构建链接通过；全应用 Debug DTCMRAM 为 116504 B / 128 KiB（88.89%）。编译仍有既有 PID C-linkage 和 volatile 自增等警告。
- 未烧录或上板。型号量程、安装方向、机械零位/限位、MIT 增益、Yaw 转矩环及真实 CAN 拥塞/断线恢复需实机验证。本次未增加整车命令来源心跳和输入仲裁。

## 2026-09-26

### 已修复

仅将 main 的四项修复适配到 RoboMaster_H7，未合入 Health、SpinMode，也未将整个 main 标记为已合并：

- `f40293d`：CAN 接收临时缓冲扩为 64 字节以容纳 HAL 的 DLC 复制；拒绝远程、扩展及 FD 帧，向设备交付的 Classic 数据长度最多 8 字节。保留目标分支的发送统计。
- `2990b25`：USB 使用自有发送缓冲，忙时立即返回；CDC 桥接传递实际接收指针，修复重新枚举后的缓冲错配。
- `8513892`：OSPI 返回 HAL 提交结果，失败时不继续 DMA；Flash 清理失败状态并停止后续步骤，Quad WIP 读取补齐数据阶段。
- `92e6314`（选择性移植）：修正 BMI088 加速度计软复位地址，确保零值配置执行写入及回读；Flash 按三字节校验本次 JEDEC 响应，排除残留数据。保留目标分支的有限重试、PWM 失败保护、初始化期 SPI 回调，以及 READY/DEGRADED/FATAL 启动分级。

### 验证

- 新增 Communication 17 组、Initialization 7 组，连同既有 7 个工程共 43 项主机测试全部通过；启动回归覆盖 32 种失败组合。
- 默认 Debug、Gimbal/Chassis/Shoot 全开的 Debug 与 Release 固件均构建链接通过；`git diff --check` 通过。
- 默认 Debug 的 DTCMRAM 为 108080 B / 128 KiB（82.46%）；三应用全开 Debug 为 116584 B（88.95%）。USB 自有发送缓冲增加 512 B 静态存储。
- 编译仍有 PID C-linkage、volatile 自增和聚合初始化等警告。未烧录或实机验证；真实通信、USB 关中断时长和 Flash DMA 接受后的硬件异常恢复仍需板测。

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

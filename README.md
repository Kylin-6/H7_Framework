# H7_Framework

> **第一次使用 H7_RM？从这里开始**
>
> 1. 阅读 [30～60 分钟快速上手](docs/GETTING_STARTED.md)，先构建并找到控制任务。
> 2. 看 [新人控制数据流图](Assets/Architecture/H7_RM_GettingStarted.svg)（[交互版](Assets/Architecture/H7_RM_GettingStarted.html)），理解控制、姿态和在线监控三条链。
> 3. 需要完整工程分层时，看 [H7_RM / H7_BSP 总览图](Assets/Architecture/H7_BSP.svg)（[交互版](Assets/Architecture/H7_BSP.html)）。
> 4. 具体开发再进入下方各模块 reference；快速上手不替代接口与硬件约定。

面向达妙 MC-02 开发板的 STM32H7 板级支持与机器人控制框架，基于 **STM32H723VGT6 / Cortex-M7 / 480 MHz**。工程围绕外设通信、设备驱动、控制与估计算法、系统服务组织代码，供机器人项目组合和复用。

底层使用 STM32CubeMX、HAL 与 FreeRTOS，任务接口采用 CMSIS-RTOS V2，构建使用 CMake + Ninja。用户层保持 C 风格运算、结构体与自由函数，设备和算法保留简洁的 `Class_` 封装。

内部物理量统一使用 SI：角度 rad、角速度 rad/s、线速度 m/s、转矩 N·m。
degree 仅用于机械标定输入、调试显示和外部协议边界；进入控制链时转换为 rad。

> **打开 `H7_BSP.ioc` 遇到版本迁移提示时，选择 Continue，不要选择 Migrate。** 迁移并重新生成可能使 `Middlewares/` 中的 FreeRTOS 与现有 SystemView 适配不兼容。请保持项目原有固件包，详见 [CubeMX 与构建边界](#cubemx-与构建边界)。

[整体架构](#整体架构) · [通信与外设](#通信与外设-bsp) · [设备层](#设备层) · [算法层](#算法层) · [系统服务](#系统服务) · [可靠性与降级边界](#可靠性与降级边界) · [接入方式](#接入方式) · [构建与调试](#构建与调试)

源码阅读：[全项目源码导航与调用约束](docs/CODE_GUIDE.md)。

核心专篇：[BSP 开发指南](User_File/Middleware/BSP/README.md) · [Message Center](User_File/System/MessageCenter/README.md) · [Application 开发指南](User_File/Application/README.md)

## 整体架构

框架以模块职责划分边界：BSP 处理外设收发，Device 处理设备协议与状态，Algorithm 提供计算组件，System 提供共享服务；Task 和 Application 负责调度与业务组合。

![H7_BSP 整体架构](Assets/Architecture/H7_BSP.svg)

> [打开交互式 H7_BSP 架构图](Assets/Architecture/H7_BSP.html)：支持亮/暗主题、搜索、聚焦、关系追踪和导出；可维护源为 [H7_BSP.architecture.json](Assets/Architecture/H7_BSP.architecture.json)。

| 层次 | 职责 | 入口 |
| --- | --- | --- |
| Application / Task | 组织控制逻辑、任务周期与模块协作 | [Application 指南](User_File/Application/README.md)、[Task](User_File/Task) |
| Device | 封装电机、板载器件与外接工具 | [Device 函数使用指南](User_File/Device/README.md) |
| Algorithm | 提供控制、观测、滤波、数学与调度辅助组件 | [Algorithm](User_File/Middleware/Algorithm) |
| System | 统一初始化、回调、时间戳与调试服务 | [System](User_File/System) |
| BSP | 管理外设实例、缓冲区、收发与回调注册 | [BSP 指南](User_File/Middleware/BSP/README.md) |
| HAL / RTOS / 工程配置 | 外设初始化、任务调度、内存布局与构建 | [Core](Core)、[User_Config](User_Config)、[CMakeLists.txt](CMakeLists.txt) |

### 工程目录

```text
User_File/
├── Application/            输入适配、命令仲裁与机构控制
├── Task/                   CMSIS-RTOS V2 任务入口
├── Device/Onboard/         板载设备
├── Device/Peripheral/      外接电机与调试工具
├── Middleware/Algorithm/   数学、控制、观测与滤波组件
├── Middleware/BSP/         外设管理与收发接口
└── System/                 初始化、回调、时间戳、参数与调试
Core/                       CubeMX 生成的启动、外设与 RTOS 配置
Drivers/                    HAL / CMSIS 驱动
Middlewares/                FreeRTOS、USB Device、CMSIS-DSP 等依赖
USB_DEVICE/                 USB CDC 设备配置
User_Config/                链接脚本、FreeRTOS 补丁与 Ozone 配置
SystemView/                 SEGGER SystemView 与 RTT
sysid/                      系统辨识数据、脚本与报告
```

## 通信与外设 BSP

BSP 以外设管理对象和接口函数承接 HAL，设备层通过注册回调与收发接口使用总线资源。
启动顺序、DMA 内存、并发模型、错误处理和扩展检查表见
[BSP 开发指南](User_File/Middleware/BSP/README.md)。

| 模块 | 提供的能力 | 使用入口 |
| --- | --- | --- |
| CAN / FDCAN | 按总线与 ID 分发接收；命令队列与最新周期帧两条发送通道 | [bsp_can.h](User_File/Middleware/BSP/CAN/bsp_can.h) |
| UART | DMA + IDLE 不定长接收、双缓冲、错误恢复；DMA 发送复制到专用缓冲并返回提交状态 | [bsp_uart.h](User_File/Middleware/BSP/UART/bsp_uart.h) |
| SPI | 外设管理、片选、收发缓冲与完成回调 | [SPI](User_File/Middleware/BSP/SPI) |
| USB | CDC 收发封装，供调试通信组件使用 | [USB](User_File/Middleware/BSP/USB) |
| OSPI | 外部存储器收发与自动轮询接口 | [OSPI](User_File/Middleware/BSP/OSPI) |
| ADC | 校准、DMA 采样与采样缓冲管理 | [ADC](User_File/Middleware/BSP/ADC) |

CAN 的两条发送通道适用于不同数据语义：

- `CAN_Tx_Perform()` 更新 `(FDCAN, ID)` 对应的周期槽，同一键保留最新数据，适合连续控制目标。
- `CAN_Tx_Submit()` 将离散命令复制到 FDCAN1/2/3 各自的 FIFO；同总线按序重试，单总线拥塞不阻断其他总线或周期槽。
- `CanTxTask` 每 1 ms 调用 `BSP_CAN_SendAsync()` / `BSP_CAN_SendPer()`。三路 FDCAN 已启用硬件 Auto Retransmission；软件队列只重试 HAL 尚未接受的帧。软件提交成功、写入硬件 FIFO、总线发送、对端收到和设备执行是不同阶段；调用方须检查提交结果。

CAN 接收回调在中断上下文执行。UART 的 DMA 接收须同时具备 CubeMX 的 RX DMA 配置和 BSP 管理入口，接入新端口时需同步核对。

`UART_Transmit_Data()` 在有 TX DMA 时，将数据复制到该端口位于 `.dma_buffer` 的专用发送缓冲；调用返回后，调用方可复用原始数据。UART 或 DMA 忙时返回 `HAL_BUSY`，不覆盖正在发送的内容；启动失败返回对应 HAL 状态，由调用方决定重试。无 TX DMA 的端口保留阻塞发送路径。

## 设备层

设备层维护协议编解码、状态、反馈与设备操作，复用 BSP 通信接口和系统时间服务。

### 电机组件

| 驱动 | 支持范围 | 接入说明 |
| --- | --- | --- |
| DJI | M2006/C610、M3508/C620、GM6020；反馈、控制环与分组发送 | [DJI 电机驱动](User_File/Device/Peripheral/Motor/DJImotor/dji_motor.md) |
| 达妙 | MIT、位置-速度、速度、力位混控接口；实际模式取决于型号与固件 | [达妙电机驱动](User_File/Device/Peripheral/Motor/DMmotor/dmmotor.md) |

电机型号、CAN ID、反馈源、方向、映射范围和控制参数由使用方配置；应用层负责控制周期、目标生成与输出边界。

达妙动作/模式请求接口返回 `bool`，表示是否成功提交到软件发送通道。提交失败时保留相应状态，调用方可据此重试；达妙置零仅在提交成功后重置位置展开状态。返回成功不代表电机已经执行或确认命令。

达妙反馈以 `(FDCAN, master_id)` 注册接收入口，并用反馈首字节低四位匹配 `can_id`；电机 ID 使用非零 8 位值，高四位仍用于发送 ID。只有总线、ID、DLC 和节点号全部合法的运动反馈才刷新在线状态。Application 用 `RequestEnabled(bool)` 指定输出许可；DMMotor 在首次请求或状态边沿执行协议动作，失能请求立即尝试覆盖安全周期目标，相同状态重复请求不执行收发；具体语义见 [达妙电机驱动](User_File/Device/Peripheral/Motor/DMmotor/dmmotor.md)，由 `StatusTask` 的 `ServiceAll()` 以 100 Hz 补交失败项并依据新鲜反馈维护 Enable/Disable 协议状态。Daemon 只判断活性，DMMotor 自己在超时后覆盖安全目标并提交失能；Gimbal 根据初始化、功能模式和 INS 有效性决定控制许可，电机快照的 ready 只用于姿态捕获与恢复。

### 板载设备与外接工具

| 组件 | 功能 |
| --- | --- |
| BMI088 | 加速度计、陀螺仪、温控与姿态组件；当前采集采用 FIFO / SPI DMA，姿态使用 VQF |
| W25Q64JV | 基于 OSPI 的外部 Flash 驱动 |
| Power | 板载电源输出控制与 ADC 电压采样 |
| WS2812 / Buzzer / Key | 灯效、蜂鸣器与按键处理 |
| EricTool | USB / UART JustFloat 输出与 `variable:value#` 文本指令解析；UART 输出返回发送状态 |

板载组件位于 [Onboard](User_File/Device/Onboard)，外接组件位于 [Peripheral](User_File/Device/Peripheral)。硬件资源绑定和设备初始化集中在 [Init.cpp](User_File/System/Init/Init.cpp)。

EricTool 的 USB/UART 解析均只读取回调传入的缓冲区及有效长度，通过字符串指针字典匹配变量名；非法帧返回索引 `-1`、值 `0`，不会沿用上一帧结果。当前保持首帧语义，第一个 `#` 后的字节忽略，不提供跨回调拼帧。UART 周期输出的返回值沿用 HAL 状态，应用需处理忙或失败。

## 算法层

算法按用途独立组织，设备和应用通过输入、参数与计算接口组合使用。现有源码在根 CMake 中显式注册；组件进入构建不代表已经接入具体控制闭环。

| 分类 | 组件 | 内容 |
| --- | --- | --- |
| 控制 | [PID](User_File/Middleware/Algorithm/PID) | PID 与前馈、积分分离/变速积分、微分先行及可选 D 支路一阶 IIR 低通 |
| 控制 | [SMC](User_File/Middleware/Algorithm/SMC) | 单轴二阶对象滑模控制，线性滑模面与饱和边界层 |
| 轨迹 | [Trajectory](User_File/Middleware/Algorithm/Trajectory) | 单轴三阶 S 曲线，位置/速度目标，限制速度、加速度和 jerk，支持运动中改目标 |
| 模糊推理 | [Fuzzy](User_File/Middleware/Algorithm/Fuzzy) | 双输入、多输出零阶 Sugeno 推理，完整规则表与分片双线性插值 |
| 观测 | [DOB](User_File/Middleware/Algorithm/DOB) | 一阶名义模型、零阶保持离散与 Q 滤波扰动估计 |
| 状态估计 | [Kalman](User_File/Middleware/Algorithm/Filter/Kalman)、[EKF](User_File/Middleware/Algorithm/Filter/EKF) | 线性与扩展卡尔曼滤波组件 |
| 姿态估计 | [VQF](User_File/Middleware/Algorithm/Filter/VQF) | 姿态与陀螺仪零偏估计 |
| 信号滤波 | [Frequency](User_File/Middleware/Algorithm/Filter/Frequency)、[IIR](User_File/Middleware/Algorithm/Filter/IIR) | FIR 频率滤波与 IIR 低通、陷波等组件 |
| 自适应滤波 | [OneEuro](User_File/Middleware/Algorithm/Filter/OneEuro) | 标量 One Euro 低通，根据变化速率调节截止频率 |
| 多项式滤波与微分 | [Polynomial](User_File/Middleware/Algorithm/Filter/Polynomial) | 0～3 阶等权拟合，输出通用标量平滑值及一、二、三阶时间导数 |
| 数学 | [Basic](User_File/Middleware/Algorithm/Basic)、[Complex](User_File/Middleware/Algorithm/Complex)、[Matrix](User_File/Middleware/Algorithm/Matrix)、[Quaternion](User_File/Middleware/Algorithm/Quaternion) | 基础运算、复数、定长矩阵与姿态表示转换 |
| 辅助 | [Slope](User_File/Middleware/Algorithm/Slope)、[FSM](User_File/Middleware/Algorithm/FSM)、[Queue](User_File/Middleware/Algorithm/Queue)、[Pulse](User_File/Middleware/Algorithm/Pulse) | 斜坡、状态机、队列与周期分频 |

使用算法时需要明确量纲、采样周期、状态初始化和输出限幅。模型相关约定以模块头文件为准，例如 DOB 使用 `y[k]` 与上一周期实际输入 `u[k-1]`，SMC 由调用方提供同一时刻的状态及其导数。

### 控制与轨迹约定

- **PID**：死区作用于有效误差，不修改调用者目标；积分在本周期累加后限幅，支持负 `Ki`，`Ki=0` 时清空积分。积分限幅为零表示不限制积分，积分分离和变速积分的阈值约定见头文件。
- **D 支路滤波**：`D_Filter_Cutoff` 使用 Hz，默认 `0` 关闭；与 `D_First` 微分先行独立配置。先滤波差分速率，再乘 `Kd`；DJI 的 `PID_InitTypeDef` 配置已透传该字段。首次启用或切换微分来源时滤波状态从零开始，持续启用且来源不变时保留滤波值。PID 参数更新不自动清空全部历史状态，死区也不保证总输出为零。
- **Trajectory**：独立于原有 Slope，一个对象管理一个轴。位置目标以零速度、零加速度到达；速度目标到达后保持匀速，设置零速度可平滑停止。目标在下一周期从当前规划的 `p/v/a` 接续，重复目标不重新规划。模块不分配堆内存、不创建任务，不保证时间最优或多轴同步；制动距离内改目标允许必要的越过与返回。

### 滤波、估计与模糊推理约定

- **One Euro**：固定周期标量输入，以首帧对齐初值；最低截止频率、速率系数 `Beta` 与导数截止频率可配置。周期或参数改变时重新初始化。
- **Polynomial**：默认二阶、支持 0～3 阶，窗口最多 33 点，在最新样本时刻求值。0 阶为移动平均；未收满窗口时原量直通、导数清零且 `Ready=false`，高于拟合阶数的导数恒为零。调用方负责等间隔新样本、量纲与角度展开，缺测后重置。
- **Kalman / EKF**：每周期先预测，缺测时跳过测量更新；更新接口返回 `bool`，求逆失败或更新结果非有限时保留当前 X/P 并清零 K，恢复有效测量后可继续更新。失败保护限于测量更新，预测和模型输入由调用方保证有效。
- **Matrix 求逆**：采用缩放部分选主元；`Matrix_Compare_Epsilon` 在此接口中是无量纲主元比例阈值。输入、消元过程或结果非有限时失败；返回零矩阵作为占位，调用方应通过 `Get_Inverse(&success)` 区分失败。
- **Sugeno**：调用方提供有序节点和完整规则表，节点/规则在使用期间保持有效且只读；输入超范围时保持边界值。输入缩放、微分、规则设计及 PID 增益映射由应用负责，库中没有预设的电机或云台控制规则。

## 系统服务

| 服务 | 作用 | 入口 |
| --- | --- | --- |
| 初始化 | 绑定 BSP、配置设备与建立系统服务 | [System/Init](User_File/System/Init) |
| 回调分发 | 将 HAL 回调转交外设与设备处理逻辑 | [System/callback](User_File/System/callback) |
| 时间戳 | 提供统一微秒时间，供周期测量与超时判断使用 | [System/Timestamp](User_File/System/Timestamp) |
| 参数配置 | 集中维护当前 IMU 采样、姿态与零偏估计参数 | [System/IMU](User_File/System/IMU) |
| 消息中心 | 静态 Latest-Value Topic 与事件 FIFO | [System/MessageCenter](User_File/System/MessageCenter) |
| 在线检测 | 固定容量设备注册、Feed 与超时状态检查 | [System/Daemon](User_File/System/Daemon) |
| 板间 Transport | 构建期固定的双板命令与反馈协议 | [Transport](User_File/System/Transport/README.md) |
| 调试数据 | 导出便于 Watch、绘图与遥测读取的状态 | [System/debug](User_File/System/debug) |
| 周期与任务 | CMSIS-RTOS V2 任务入口、线程标志和周期回调 | [Task](User_File/Task) |

[main.c](Core/Src/main.c) 完成 MPU、HAL 和外设初始化后调用 `System_Init()`，随后初始化 RTOS、创建任务并启动调度。CAN 发送资源在内核初始化后建立，周期服务与设备计算按职责由任务调度。

### 消息中心

消息中心提供两种静态、类型安全的数据通道：

- `Topic<T>` 使用 Latest-Value 语义，传递连续状态和控制目标；`Publisher`/`Subscriber` 只是其无分配访问封装。
- `EventQueue<T,N>` 使用固定容量 FIFO，传递不能被最新值覆盖的离散事件；队列满时拒绝新事件并累计溢出次数。

业务类型和唯一静态通道统一定义在 [MessageCenter](User_File/System/MessageCenter)。`INS_State_Topic` 由 BMI088 链路发布，云台读取最新姿态；RobotCmd 通过 Output 发布 Gimbal、Chassis、Shoot 连续命令并汇总反馈，底盘命令在云台板由固定 Transport 送往底盘板 Topic。单发和三连发通过固定容量 `ShootEvent` FIFO 传递。完整 API、并发语义、通道所有权、示例和验证清单见 [Message Center 专篇](User_File/System/MessageCenter/README.md)。

所有正常工作时应持续收到反馈、心跳或数据流的模块优先注册静态 Daemon，只在收到合法数据时 `Feed()`。当前已接入 DM、DJI、S.BUS、有效 INS 输出、双板 Transport 及可选 Referee/VTM；`StatusTask` 每 10 ms（100 Hz）统一 `CheckAll()`，随后调用已编入的 DJI、DM 设备 `ServiceAll()`。Daemon 只负责 liveness，不负责整车停机、清错、重启、安全策略或消息路由。管理器保持 32 个固定槽位，无动态分配；当前静态应用与可选 Referee/VTM 的三板注册数为 17/8/11。电机在线查询与控制门控统一使用 Daemon 即时状态；Topic ReadFresh 仍独立判断业务数据时效，详见 [Daemon 说明](User_File/System/Daemon/README.md)。

### Application

Application 作为独立机器人业务层维护，不在 BSP 总览展开具体控制实现。当前模块、
Control_Task 调度顺序、RobotCmd 所有权、Gimbal/Chassis/Shoot 行为和新应用接入规范见
[Application 开发指南](User_File/Application/README.md)。

当前 App 主流程保持“读取输入 → 表达功能许可 → 更新目标 → 计算并提交控制 → 发布反馈”。
Gimbal 合并姿态捕获条件，并保留恢复后等待新 IMU 目标的要求；Shoot 在模式处理后统一选择拨弹外环与目标；RobotCmd 复用失联和切源时的射击事件清理。设备级掉线保护由 Motor 独立执行，WS2812 保持现有颜色缓存与刷新行为。

### 单板与双板

构建目标在编译期确定应用与任务：`SingleBoard` 的 Gimbal、Chassis、Shoot 默认关闭；关闭应用时不编译或调度对应 App，也不发布对应本地应用反馈；关闭 Shoot 时拒绝射击事件；`GimbalBoard` 运行 RobotCmd、Gimbal、Shoot；`ChassisBoard` 运行 Chassis。BoardConfig 只绑定本板硬件，TransportConfig 固定板间总线和报文号。运行时不使用 Router 或动态 Topic 路由。

单板 RobotCmd 的三个 Output 都是 LocalPublisher；云台板的底盘 Output 是 RemotePublisher。`ChassisCmd` 经 CAN 标准 ID `0x141` 到达底盘板本地 Topic，`ChassisFeedback` 经 `0x222` 返回云台板本地 Topic。两者是 8 字节 Classic CAN 最新值，接收任务按实际 RX 时间和序号校验，底盘命令超过 100 ms 变为 `ZERO_FORCE`。INS、Gimbal、Shoot 的 1 kHz 板内路径不经过 Transport。协议和接线见 [双板 Transport](User_File/System/Transport/README.md)。

## 可靠性与降级边界

### 启动状态

`System_Init()` 不用单一成功标志掩盖部分设备失败，而是同时公开总体状态与失败位图：

| 状态 | 含义 | 当前处理 |
| --- | --- | --- |
| `SYSTEM_INIT_READY` | 必需与可选模块全部初始化成功 | 正常启动控制任务 |
| `SYSTEM_INIT_DEGRADED` | BMI088、W25Q64 或 ADC1 等可选设备失败 | 其余模块继续运行；失败功能保持禁用 |
| `SYSTEM_INIT_FATAL` | TIM4/TIM5 等控制时基失败 | `Control_Task` 不进入控制循环 |

失败来源通过 `System_Init_GetFailureMask()` 的 `TIM4 / TIM5 / BMI088 / W25Q64 / ADC1` 位读取。`init_finished` 仅表示初始化流程已经结束，不表示所有模块均可用。BMI088 只有在 Accel/Gyro 芯片 ID 与配置回读均成功后才启动 FIFO；W25Q64 的 JEDEC ID 最多尝试 5 次，识别失败后读写和内存映射保持禁用，避免缺件时无限阻塞上电。

### 设备状态语义

设备层统一使用四个维度，应用不应把“收到过反馈”直接当作“允许输出”：

| 查询 | 语义 |
| --- | --- |
| `Online` | 在设备规定的超时窗口内收到过合法反馈 |
| `RequestedEnabled` | 电机的 Application 输出许可；DJI 只有软件 gate，DM 另有协议实际使能反馈 |
| `DataValid` | 当前反馈可供上层使用；现有驱动通常要求 Online |
| `Ready` | 电机初始化、请求使能且反馈新鲜；DM 还要求协议报告已使能且无故障 |

`Daemon` 只负责时间窗及在线/离线跃迁。设备收到完整合法反馈后自行 `Feed()`，`StatusTask` 每 10 ms 统一 `CheckAll()`，随后执行已编入的电机设备安全/协议服务。基础掉线保护由 Device 独立覆盖安全目标及执行失能，不依赖 App 持续调用控制接口。它不决定全车停机、云台 READY、消息路由或故障上报；业务安全策略仍由拥有设备的 Application 决定，并需实机拔线验证时限。

### 数据新鲜度、发送与可观测性

- `Topic<T>::ReadFresh()` 用发布时间戳拒绝过期数据。云台对 INS 使用 10 ms 新鲜度门限；失效时向两轴达妙请求失能，电机立即发布安全 MIT 输出；INS 恢复且两轴 ready 后捕获当前姿态，IMU 模式等待新目标。见 [云台说明](User_File/Application/Gimbal/README.md)。
- 连续控制目标走 `CAN_Tx_Perform()`，同一 `(FDCAN, ID)` 只保留最新值；使能、失能、清错和模式设置走 `CAN_Tx_Submit()` FIFO。软件接收成功、写入硬件 FIFO 和设备实际执行是三个不同阶段。
- `BSP_CAN_GetTxStats()` 提供命令队列满、周期槽满、硬件 FIFO 满和 HAL 发送失败的饱和计数快照。计数只提供证据，不自动改变调度或执行安全策略。
- 主机测试可以确认协议编解码、ID/DLC 隔离、超时边界、队列溢出和数据新鲜度；真实波特率/采样点、终端电阻、总线仲裁、供电时序、电机参数和 EMC 必须在目标板上确认。

当前框架尚未启用独立 IWDG，也没有通用的整车安全策略联动和复位原因遥测。接入整机前至少应完成遥控器失联互锁、关键设备拔线、上电仲裁、跌压重启和长跑水位检查，不能把主机回归通过等同于整机安全验收。

## 接入方式

### 添加设备或通信协议

- 在 `Device/Onboard` 或 `Device/Peripheral` 中放置设备实现，参照同层组件组织配置、状态和接口。
- 复用现有 BSP 管理对象与回调注册；设备层负责协议解析，任务或应用层负责业务处理和控制目标。
- 在显式 `Init()` 中绑定资源，按依赖顺序接入系统初始化；需要 RTOS 对象的部分放在内核初始化之后。
- 按需求接入周期回调或任务，明确 ISR 与任务边界、缓冲区所有权和数据有效期。
- 在根 `CMakeLists.txt` 注册源码与 include 路径，并检查收发结果、超时和实际硬件行为。

### 组合控制与算法

- 在应用层组织目标、反馈和状态，调用设备接口及算法组件；已有数学与时间服务优先复用。
- 按模块约定初始化模型与参数，在确定的采样周期内更新输入、计算输出并提交给设备。
- 控制计算、通信发送和调试输出按实时性分配，接入后观察任务栈水位、周期与执行时间。

### 代码风格

沿用同层模块的 `Struct_`、`Enum_`、`Class_` 命名和文件组织，内部优先使用直接的 C 风格运算。保留已有必要模板与类接口，新模块避免无必要的抽象层、动态分配和重复实现。

硬件操作放在 `Init()` 中，构造函数不访问硬件。C 调用入口通过 `extern "C"` 暴露，C 可见头文件保持兼容。任务与同步接口使用 CMSIS-RTOS V2，中断内调用 RTOS API 前核对优先级和接口限制。

## 工程基础

### DMA 与内存布局

项目使用自有 [h7_memory.ld](User_Config/Linker/h7_memory.ld)，由 [h7_linker.cmake](User_Config/Linker/h7_linker.cmake) 选择：

| 区域 | 地址 | 大小 | 用途 |
| --- | --- | --- | --- |
| `DTCMRAM` | `0x20000000` | 128 KiB | 普通数据、栈、部分 FreeRTOS heap；DMA1/DMA2 不可直接访问 |
| `RAM_DMA` | `0x24000000` | 64 KiB | `.dma_buffer`；MPU 配置为共享、不可缓存 |
| `RAM_D1` | `0x24010000` | 256 KiB | `.ram_d1_data` 与部分 FreeRTOS heap |
| `RAM_D2` / `RAM_D3` | `0x30000000` / `0x38000000` | 32 / 16 KiB | 其余 SRAM，使用时核对对应 DMA 的可达性 |

DMA1/DMA2 缓冲区应放入 `.dma_buffer`，并核对对齐、生命周期和传输长度；BDMA 等控制器需要单独确认内存可达性。`RAM_DMA` 的地址与大小必须与 `main.c` / `.ioc` 中的 MPU 配置一致，链接脚本包含一致性断言。

UART 的专用 TX 缓冲随管理对象放在该不可缓存区域，CPU 复制完成后通过内存屏障再启动 DMA；该路径无需额外清理 D-Cache。此约定依赖当前链接布局与 MPU 配置，修改内存属性时须一并复核。

FreeRTOS 使用 `heap_5`，默认总量 64 KiB，分为 **48 KiB DTCMRAM + 16 KiB RAM_D1**。DTCM 分区由 `H7_FREERTOS_DTCM_HEAP_SIZE` 配置，heap 与 port 补丁位于 [FreeRTOS_Patch](User_Config/FreeRTOS_Patch)。

### CubeMX 与构建边界

- **打开 IOC 时选择 Continue，禁止直接使用 Migrate 升级本工程。** 当前 [H7_BSP.ioc](H7_BSP.ioc) 记录 STM32CubeMX **6.15.0**、STM32Cube FW_H7 **V1.12.1**，仓库内 FreeRTOS 为 **V10.3.1**。缺少原固件包时先安装对应版本，再继续打开。
- `Migrate` 会迁移项目使用的数据库与固件版本；重新生成时可能替换 `Middlewares/` 中的 FreeRTOS，使其与现有 SystemView 跟踪宏、RTOS 适配和 [port 补丁](User_Config/FreeRTOS_Patch/port_patched.c) 不兼容。需要升级时应配套调整并验证这些组件，不能仅靠迁移后编译通过判断跟踪功能正常。迁移选项含义见 [ST CubeMX 官方说明](https://dev.st.com/stm32cube-docs/stm32cubemx/6.18.1/en/docs/markup/CubeMX_UserManual/chapters/04_4_stm32cubemx_user_interface.html)。
- 外设配置入口为 [H7_BSP.ioc](H7_BSP.ioc)，生成代码的用户修改放在 `USER CODE BEGIN/END` 区域。
- 用户源码与 include 路径由根 [CMakeLists.txt](CMakeLists.txt) 显式维护；链接布局、RTOS 补丁维护在 `User_Config/`。
- CubeMX 重生成后重新 configure/build，检查用户集成、链接脚本选择和补丁接入。
- 工程链接 CMSIS-DSP Cortex-M7 浮点库，C++ 工具链禁用异常和 RTTI。

## 构建与调试

### 推荐 VS Code 插件

**优先推荐 [EmberProbe - MCU Flash & Debug](https://marketplace.visualstudio.com/items?itemName=BakeSheep.emberprobe)**（[GitHub 仓库](https://github.com/BakeSheep/EmberProbe-MCU-Flash-Debug)），作为本项目烧录、断点调试、实时变量观测与 ELF 分析入口。构建与源码阅读搭配 STM32CubeIDE、CMake Tools 和 STM32Cube clangd，C/C++ 格式化推荐 Clang-Format 并使用仓库 `.clang-format`；完整清单及板型 ELF 选择步骤见 [VS Code 插件推荐](docs/VSCODE_EXTENSIONS.md)。

### 环境与构建

准备 CMake 3.22 或更高版本、Ninja 和 GNU Arm 工具链，确保 `arm-none-eabi-gcc` / `arm-none-eabi-g++` 等命令可用。在仓库根目录执行：

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

产物为 `build/Debug/H7_Framework.elf`，链接映射为同目录下的 `H7_Framework.map`。Release 使用对应 preset：

```powershell
cmake --preset Release
cmake --build --preset Release
```

[CMakePresets.json](CMakePresets.json) 保留 Debug/Release 配置；[CMakeUserPresets.json](CMakeUserPresets.json) 提供 SingleBoard/GimbalBoard/ChassisBoard。Debug 使用 `-Og -g3`，Release 使用 `-Os -g0`。所有配置均生成同名的 ELF 和 map，供构建分析器读取：

| Preset | 构建目录 | ELF / map 文件名（不含扩展名） |
| --- | --- | --- |
| Debug | `build/Debug` | `H7_Framework` |
| Release | `build/Release` | `H7_Framework` |
| SingleBoard | `build/SingleBoard` | `H7_Framework` |
| GimbalBoard | `build/GimbalBoard` | `H7_Framework` |
| ChassisBoard | `build/ChassisBoard` | `H7_Framework` |

### 烧录与观察

烧录使用 VS Code 的 EmberProbe: Flash & Debug 插件。先构建所需 preset，再选择对应 `build/<preset>/H7_Framework.elf`，并核对探针与 STM32H723 目标配置；不要选择旧名称的构建产物。仓库不再维护独立烧录脚本。

保留 [Ozone 工程](H7_BSP.jdebug) 作为 J-Link 源码调试入口（默认加载 Debug 固件），以及 [Ozone DAPLink 配置](User_Config/ozone_daplink.cfg)。

- Ozone / GDB：观察设备反馈、算法状态、系统调试数据与任务栈水位。
- SystemView / RTT：观察任务调度、中断和运行时信息。
- EricTool：通过 USB / UART 输出数据；`TransportTask` 中保留了 USB 周期输出的使用示例。
- [sysid](sysid/README.md)：系统辨识数据、采集分析脚本与实验报告。

## 文档与参考

- [BSP 开发指南](User_File/Middleware/BSP/README.md) · [Message Center](User_File/System/MessageCenter/README.md) · [Application 开发指南](User_File/Application/README.md)。
- [DJI 电机驱动](User_File/Device/Peripheral/Motor/DJImotor/dji_motor.md) · [达妙电机驱动](User_File/Device/Peripheral/Motor/DMmotor/dmmotor.md) · [更新记录](docs/CHANGELOG.md)。
- [FreeRTOS heap memory management](https://www.freertos.org/Documentation/02-Kernel/02-Kernel-features/09-Memory-management/01-Memory-management)。

## 致谢

感谢 MermaidFAR 开源并提供本项目所基于的 [H7_BSP](https://github.com/MermaidFAR/H7_BSP) 基础工程。

本框架的分层设计、设备抽象与工程组织参考了[湖南大学 RoboMaster 跃鹿战队 `basic_framework`](https://github.com/HNUYueLuRM/basic_framework)、中国科学技术大学 RoboWalker 的开源框架 [`damiao_mc02_bsp`](https://github.com/yssickjgd/damiao_mc02_bsp)，以及 [Meta-Team 的 `Meta-Embedded-NG`](https://github.com/Meta-Team/Meta-Embedded-NG)。

感谢 xrobot-org 开源并分享 [`libxr`](https://github.com/xrobot-org/libxr)。

<a id="维护架构图"></a>

<details>
<summary>维护架构图</summary>

两张图共用已有 **Archify** 工具链（当前产物生成版本 `2.17.0-dev.1`），仓库没有架构图专用 `package.json` / npm script，也没有 Python 生成器：

- [H7_BSP.architecture.json](Assets/Architecture/H7_BSP.architecture.json)：工程分层总览，保留原文件名。
- [H7_RM_GettingStarted.architecture.json](Assets/Architecture/H7_RM_GettingStarted.architecture.json)：新人控制、姿态、在线监控三条链。
- [Export_Svg.mjs](Tools/Architecture/Export_Svg.mjs)：通过 Archify HTML 的浏览器导出接口生成 SVG，保留字体、主题和拓扑。

**只修改 JSON 图源，不手改 SVG / HTML。** 图源使用 Archify `schemas/architecture.schema.json` 和 `schemas/common.schema.json`：`schema_version: 1`、`diagram_type: architecture`，主要字段为 `meta`、`components`、`boundaries`、`connections`、`cards`。
component type 是固定枚举；本工程用 `meta.legend.entries.<type>.label` 显示嵌入式层次，不沿用 Web 图例。方向相反的关系用两条连接表达。
`meta.repository.revision` 固定源码证据版本；更新架构时同步为已核对的 commit，`sources` 指向该版本中的真实文件。

准备 Node.js 18+、已有 Archify skill 目录和 Chrome / Chromium（SVG 导出及浏览器验证需要；必要时用 `ARCHIFY_CHROME` 指定浏览器路径）。在**仓库根目录**执行以下命令，先将 `ARCHIFY_ROOT` 改为本机 skill 目录：

```bash
ARCHIFY_ROOT=/path/to/archify

# 每份 JSON 独立校验并生成交互式 HTML
for diagram in H7_BSP H7_RM_GettingStarted; do
  node "$ARCHIFY_ROOT/bin/archify.mjs" validate architecture \
    "Assets/Architecture/$diagram.architecture.json" \
    --quality showcase --repo-root . --json || break
  node "$ARCHIFY_ROOT/bin/archify.mjs" deliver architecture \
    "Assets/Architecture/$diagram.architecture.json" \
    "Assets/Architecture/$diagram.html" \
    --quality showcase --repo-root . --json || break
done

# 从已成功交付的 HTML 生成 SVG
for diagram in H7_BSP H7_RM_GettingStarted; do
  node Tools/Architecture/Export_Svg.mjs "$ARCHIFY_ROOT" \
    "Assets/Architecture/$diagram.html" \
    "Assets/Architecture/$diagram.svg" || break
done

# 记录多分辨率浏览器证据
for diagram in H7_BSP H7_RM_GettingStarted; do
  node "$ARCHIFY_ROOT/bin/archify.mjs" visual-check \
    "Assets/Architecture/$diagram.html" --json || break
done
```

校验失败时先修复 JSON，再重新生成；不要从失败交付后保留的旧 HTML 导出 SVG。
HTML 支持亮/暗主题、搜索、聚焦、关系追踪、三条链的引导视图和导出。
两份 JSON 及对应的 SVG、HTML **一并提交 git**，方便 GitHub 阅读和离线交互。
浏览器 QA 生成的截图与 receipt 是本地验证证据，不作为图源；交付前检查两种主题、文字和箭头，并运行 `git diff --check`。

</details>

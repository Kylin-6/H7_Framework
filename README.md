# H7_BSP

基于 STM32H723ZG 的 RoboMaster 竞赛机器人板级支持包。使用 STM32CubeMX 生成 HAL 外设初始化作为底层基础，在 `User_File/` 中维护 C++ BSP、设备驱动、中间件算法、消息中心和 FreeRTOS 任务。

## 当前状态

| 项目 | 状态 | 说明 |
|---|---|---|
| 构建 | ✅ 已通过 | `cmake --build --preset Debug` 生成 `build/Debug/H7_BSP.elf` |
| MCU | ✅ 已配置 | STM32H723ZG，Flash 1024K，DTCMRAM / RAM_D1 / RAM_D2 / RAM_D3 分区已在链接脚本中定义 |
| RTOS | ✅ 已接入 | FreeRTOS + CMSIS-RTOS V2；`heap_5` 按 48 KiB DTCMRAM + 16 KiB RAM_D1 双区配置 |
| SystemView | ✅ 已接入 | SEGGER SystemView + RTT；`port_patched.c` 替换原始 FreeRTOS port |
| 消息中心 | ✅ 已实现 | 双通道：静态 `Topic<T>`（INS 高频）+ 动态 Pub/Sub（应用命令/反馈）|
| Daemon | ✅ 已实现 | 固定容量、无堆分配；`StatusTask` 每 10 ms 统一检查设备在线状态 |
| BMI088 + VQF | ✅ 已形成主链路 | FIFO → SPI DMA → `BMI088_Task` → VQF 姿态解算 → 发布 `INS_State_Topic` |
| 云台（Gimbal） | ✅ 已实现 | QD4310 双轴，Yaw 双环（角度 + 速度 PID），Pitch 内置位置环 |
| 底盘（Chassis） | ✅ 骨架已实现 | 四舵轮 AGV 运动学，默认 `CHASSIS=0` |
| 射击（Shoot） | ✅ 骨架已实现 | 摩擦轮 + 拨盘多模式，默认 `SHOOT=0` |
| DJI 电机 | ✅ 已接入 | M2006 / M3508 / GM6020，FDCAN 双通道发送 |
| 达妙电机 | ✅ 已接入 | MIT / 位置-速度 / 速度 / 力位混控 |
| CAN BSP | ✅ 已实现 | 周期槽（`CAN_Tx_Perform`）+ 异步队列（`CAN_Tx_Submit`）双通道 |
| UART BSP | ✅ 已实现 | 双缓冲 DMA 接收，接管 7 路有 RX DMA 的 UART |
| 系统辨识 | ✅ 已完成 | Yaw 速度环 + 位置环辨识，含 MATLAB/Python 脚本与实测日志 |

最近一次本地构建内存占用：

| 内存区域 | 使用量 | 总量 | 使用率 |
|---|---:|---:|---:|
| DTCMRAM | 107224 B | 128 KB | 81.81% |
| RAM_DMA | 17120 B | 64 KB | 26.12% |
| RAM_D1 | 16384 B | 256 KB | 6.25% |
| FLASH | 119792 B | 1024 KB | 11.42% |

---

## 目录结构

```text
H7_BSP/
├── Core/                          STM32CubeMX 生成代码，仅在 USER CODE 区修改
├── Drivers/                       STM32 HAL / LL / CMSIS 驱动库
├── Middlewares/                   FreeRTOS / USB Device / CMSIS-DSP
├── SystemView/                    SEGGER SystemView + RTT
├── USB_DEVICE/                    STM32 USB CDC
├── User_Config/                   工具与补丁
│   ├── FreeRTOS_Patch/            heap_5 双区配置 + patched port.c
│   └── Linker/                    用户内存段链接脚本
├── User_File/
│   ├── Application/               应用层（Gimbal / Chassis / Shoot / RobotCmd）
│   ├── Device/
│   │   ├── Onboard/               板载器件（BMI088 / WS2812 / Buzzer / Flash 等）
│   │   └── Peripheral/            外接器件（QD4310 / DJI 电机 / 达妙电机 / EricTool）
│   ├── Middleware/
│   │   ├── Algorithm/             算法库（PID / EKF / VQF / 矩阵 / 四元数 / FSM 等）
│   │   └── BSP/                   外设抽象（CAN / SPI / UART / ADC / USB / OSPI）
│   ├── System/
│   │   ├── Daemon/                设备在线检测（Feed + 超时检查）
│   │   ├── MessageCenter/         消息中心（静态 Topic + 动态 Pub/Sub）
│   │   ├── IMU/                   IMU 系统级参数配置与 INS 状态发布
│   │   ├── Init/                  系统初始化入口
│   │   ├── callback/              HAL 中断回调统一分发
│   │   ├── Timestamp/             微秒级时间戳服务
│   │   ├── debug/                 调试数据 ABI（J-Link / SystemView / RTT）
│   │   └── Storage/               Flash 存储布局定义
│   └── Task/                      FreeRTOS 任务入口
└── sysid/                         云台系统辨识数据、脚本与报告
```

---

## 快速开始

### 构建

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

输出文件：`build/Debug/H7_BSP.elf`

Release 构建：

```powershell
cmake --preset Release
cmake --build --preset Release
```

### 烧录

VS Code 任务中已配置四种烧录方式（`Ctrl+Shift+P → Tasks: Run Task`）：

| 任务名 | 工具 |
|---|---|
| Flash H7_BSP (ST-Link/OpenOCD) | OpenOCD |
| Flash H7_BSP (J-Link/Ozone) | Ozone |
| Flash H7_BSP (J-Link/CLI) | J-Link 命令行 |
| Generate J-Link Flash Script | 生成 J-Link CLI 脚本 |

### 启用应用层模块

应用层模块默认关闭，通过 CMake 选项启用：

```powershell
cmake --preset Debug -DH7_APP_GIMBAL=ON -DH7_APP_CHASSIS=ON -DH7_APP_SHOOT=ON
cmake --build --preset Debug
```

启用前必须确认对应 CAN 总线、电机 ID、机械参数和 PID 参数。

---

## 系统架构

### 启动顺序

```
main()
  ├── MPU_Config()          RAM_D1 配置为 non-cacheable，供 DMA 访问
  ├── HAL_Init()            系统时钟、外设时钟、GPIO/DMA/FDCAN/SPI/UART/TIM/ADC
  ├── System_Init()         SystemView、时间戳、SPI2/6、BMI088、外设 BSP 初始化
  ├── osKernelInitialize()
  ├── DynamicMessageCenter_Init()   消息中心初始化（调度器启动前）
  ├── Application_RegisterTopics()  注册全部 Publisher / Subscriber
  ├── BSP_CAN_ConfigInit()          CAN 发送槽与异步队列初始化
  ├── 创建任务
  └── osKernelStart()
```

### 任务列表

| 任务 | 优先级 | 栈大小 | 职责 |
|---|---|---:|---|
| `BMI088_Task` | `osPriorityHigh2` | 8 KB | FIFO 续传 + VQF 姿态解算 + 发布 INS 状态 |
| `Control_Task` | `osPriorityHigh1` | 8 KB | 1 kHz 控制环：RobotCmd / Gimbal / Chassis / Shoot |
| `Can_Tx_Task` | `osPriorityHigh` | 4 KB | 1 ms 周期：排空异步队列 + 发送周期槽 |
| `StatusTask` | `osPriorityLow` | 2 KB（静态） | 每 10 ms 调用 `DaemonManager::CheckAll()` |
| `TIM1msTask` | `osPriorityLow` | — | 静态回调表调度（1 / 10 / 50 / 128 ms 分频）|
| `TransportTask` | `osPriorityNormal` | 8 KB | USB Device 初始化 + EricTool 遥测 |
| `InsTask` | — | — | 预留，当前直接 `osThreadExit()` |
| `Storage_Task` | — | — | 预留，当前直接 `osThreadExit()` |

栈溢出保护已开启（`configCHECK_FOR_STACK_OVERFLOW = 2`），溢出时触发断点并记录任务名。

### 核心数据流

```
BMI088 陀螺仪 FIFO 中断
  → SPI2 DMA 读取
  → BMI088_Task 被唤醒
  → VQF 逐帧积分（2 kHz 陀螺仪 + 250 Hz 加速度计修正）
  → System_IMU_Publish_State()
  → MessageCenter::INS_State_Topic.Publish()

TIM4 1 ms 中断
  → osThreadFlagsSet(ControlTaskHandle, 0x0001)
  → Control_Task 被唤醒
  → RobotCmd_Update() → Gimbal_Update() → Chassis_Update() → Shoot_Update()
  → Gimbal 读取 INS_State_Topic（PRIMASK 临界区，无队列延迟）
  → QD4310 电流 / 位置命令 → CAN_Tx_Perform() → Can_Tx_Task 发送
```

---

## 消息中心

消息中心是本框架的核心通信基础设施，分为两个独立通道，各司其职。

### 静态 Topic `Topic<T>`

**文件**：`User_File/System/MessageCenter/topic.h`

用于 **高频实时状态**，典型场景：INS 姿态 → 云台控制环。

- **内存**：编译期静态对象，零堆分配，零队列
- **同步**：PRIMASK 临界区 + DMB，保证 Cortex-M7 乱序流水线上的数据可见性
- **读语义**：Latest-Value，非消费式；所有读者都能持续取得最近一次完整快照
- **类型安全**：编译期 `static_assert(std::is_trivially_copyable<T>)`
- **扩展接口**：`ReadWithMeta()` 在一次临界区内同时返回 `data`、`sequence`、`timestamp_us`、`valid`

```cpp
// 发布（BMI088_Task）
MessageCenter::INS_State_Topic.Publish(ins_state);

// 读取（Gimbal_Update，每 1 ms）
INS_State state;
if (MessageCenter::INS_State_Topic.Read(state)) { ... }

// 带元数据读取（检测是否有新帧）
auto snap = MessageCenter::INS_State_Topic.ReadWithMeta();
if (snap.valid && snap.sequence != last_seq) { ... }
```

当前已注册的静态 Topic：

| Topic | 类型 | 发布者 | 消费者 |
|---|---|---|---|
| `INS_State_Topic` | `INS_State` | `BMI088_Task` | `Gimbal_Update` |

### 动态 Pub/Sub `DynamicMessageCenter`

**文件**：`User_File/System/MessageCenter/dynamic_message_center.h/.c`

用于 **低频应用命令与反馈**，典型场景：RobotCmd → Gimbal / Chassis / Shoot。

- **内存**：FreeRTOS heap_5，启动前一次性分配，运行期链表只读
- **同步**：每个订阅者独占一个长度为 1 的 FreeRTOS Queue，`xQueueOverwrite` 保证 Latest-Value
- **读语义**：消费式，每个订阅者独立读取，互不影响
- **注册窗口**：只能在 `osKernelInitialize()` 之后、`osKernelStart()` 之前注册

```c
// 注册（Application_RegisterTopics，调度器启动前）
DynamicPublisher_t *pub = DynamicPublisher_Register("gimbal_cmd", sizeof(GimbalCmd));
DynamicSubscriber_t *sub = DynamicSubscriber_Register("gimbal_cmd", sizeof(GimbalCmd));

// 发布（RobotCmd_Update）
DynamicPublisher_Publish(pub, &cmd);

// 读取（Gimbal_Update，每 1 ms 尝试，读不到保留旧命令）
GimbalCmd cmd;
if (DynamicSubscriber_Read(sub, &cmd)) { ... }
```

当前已注册的动态 Topic：

| Topic 名 | 消息类型 | 发布者 | 订阅者 |
|---|---|---|---|
| `gimbal_cmd` | `GimbalCmd` | `RobotCmd` | `Gimbal` |
| `gimbal_feedback` | `GimbalFeedback` | `Gimbal` | `RobotCmd` |
| `chassis_cmd` | `ChassisCmd` | `RobotCmd` | `Chassis` |
| `chassis_feedback` | `ChassisFeedback` | `Chassis` | `RobotCmd` |
| `shoot_cmd` | `ShootCmd` | `RobotCmd` | `Shoot` |
| `shoot_feedback` | `ShootFeedback` | `Shoot` | `RobotCmd` |

Topic 名称集中定义在 `User_File/Application/application_topics.h`，避免字符串散落各处。

### 选用原则

| 场景 | 通道 | 原因 |
|---|---|---|
| INS → 云台，1 kHz | 静态 Topic | 零堆、零队列、PRIMASK 临界区足够短 |
| 应用命令 / 反馈，100 Hz | 动态 Pub/Sub | 每订阅者独立消费，命令所有权清晰 |
| 高频状态不得迁入动态 Queue | — | Queue 调度开销与 Latest-Value 语义不匹配 |

---

## Daemon 设备在线检测

Daemon 只回答“设备是否在线”，不负责掉线后的停机、安全策略、日志或消息路由。

- 设备收到并确认一帧合法反馈后直接调用 `Feed()`，不经过 Message Center。
- `DaemonManager` 使用固定 32 项指针数组，无 `malloc/new`，设备初始化时注册。
- `StatusTask` 以 10 ms 固定周期调用 `CheckAll()`；每个 Daemon 使用独立超时时间。
- 新建 Daemon 在收到第一帧前为 Offline，并区分 `OfflineToOnline` 和 `OnlineToOffline` 一次性状态跃迁。
- `Feed()` 与状态读取使用极短 PRIMASK 临界区，可安全跨 CAN ISR 和 StatusTask 使用。

当前首批接入 `Class_DMMotor`，反馈超时为 100 ms。Remote/DBUS 尚未实现，因此没有创建占位接入；DJI 电机和 QD4310 保留现有在线状态机制，后续按设备逐步迁移。

---

## 应用层

应用层按 `RobotCmd → Gimbal / Chassis / Shoot` 边界组织，位于 `User_File/Application/`。

### RobotCmd

命令的**唯一发布者**，同时汇总各模块反馈。后续遥控器、视觉、裁判系统等输入模块只需调用 `RobotCmd_Set*()` 写入命令；命令内容改变时才触发 `DynamicPublisher_Publish()`。

```cpp
GimbalCmd cmd;
cmd.mode = GimbalMode::IMU;
cmd.yaw_angle_rad = target_yaw;
cmd.pitch_angle_rad = target_pitch;
RobotCmd_SetGimbal(cmd);

GimbalFeedback fb;
if (RobotCmd_GetGimbalFeedback(fb)) { ... }
```

### Gimbal

- Yaw：角度外环（`INS_State_Topic` 欧拉角反馈）→ 速度内环（IMU 体轴角速度反馈）→ QD4310 电流指令
- Pitch：QD4310 内置位置环，软件只下发目标角度
- 使能重试：最多等待 2 秒（100 次 × 20 ms），超时后保留错误状态并返回，不阻塞 Chassis / Shoot 初始化
- 控制输出：只有命令非 `DISABLED` 且 Gimbal FSM 为 `READY` 时才执行控制环
- 模式切换（`DISABLED / LOCK / IMU`）在收到新命令时执行

当前 Yaw PID 参数（已系统辨识整定）：

| 环 | Kp | Ki | Kd |
|---|---:|---:|---:|
| 角度外环 | 32.00 | 0.00 | 0.00 |
| 速度内环 | 0.15 | 0.63 | 0.00 |

### Chassis（默认关闭）

四舵轮 AGV 运动学：

- 每个舵模块独立计算目标转向角和轮速
- 转向最短路径：误差超过 ±90° 时反转轮速，减少不必要的大角度转向
- 使能：`ChassisMode != ZERO_FORCE` 时启用电机输出
- 反馈：速度低通滤波后发布（α = 0.032258）

启用：`cmake --preset Debug -DH7_APP_CHASSIS=ON`，并标定 `Chassis_Steer_Offset_Deg[4]`。

### Shoot（默认关闭）

| 模式 | 行为 |
|---|---|
| `STOP` | 拨盘零速 |
| `SINGLE` | 拨动一颗子弹角度 |
| `TRIPLE` | 拨动三颗子弹角度 |
| `BURST` | 速度环连发，速度由 `shoot_rate_hz` 计算 |
| `REVERSE` | 反转退弹 |

启用：`cmake --preset Debug -DH7_APP_SHOOT=ON`，并确认摩擦轮和拨盘电机 ID。

---

## 硬件抽象层

### CAN BSP

双通道发送架构：

| 通道 | 接口 | 适用场景 |
|---|---|---|
| 周期槽 | `CAN_Tx_Perform()` + `BSP_CAN_SendPer()` | 电机控制帧，1 ms 周期精确发出 |
| 异步队列 | `CAN_Tx_Submit()` + `BSP_CAN_SendAsync()` | 使能、复位等低频非实时命令 |

接收注册：`BSP_CAN_RegisterCallback(can_id, hfdcan, callback, context)`，注册键为 `(hfdcan, can_id)`，不同总线可注册相同 ID。

FDCAN Message RAM 三路均匀分配：偏移量 0 / 853 / 1706，消除 FDCAN2 重叠问题。

### UART BSP

双缓冲 DMA 接收，接管 7 路有 RX DMA 的 UART（USART1/2/3、UART5、USART6、UART7、USART10）：

- `HAL_UARTEx_ReceiveToIdle_DMA`：IDLE 中断触发双缓冲切换，接收不定长帧不丢数据
- `UART_Init(huart, callback)`：`nullptr` 退化为轮询模式
- UART5 无 TX DMA，发送自动回退阻塞模式
- 缓冲区放入 `.dma_buffer` 段，落在 RAM_D1，避免 DTCMRAM DMA 不可达问题

### SPI BSP

- SPI1~5：DMA 全双工收发，完成回调中拉回片选、记录时间戳、调用业务回调
- SPI6：因 BDMA 内存限制，使用阻塞传输
- SPI2：服务 BMI088（加速度计 + 陀螺仪）
- 片选由 BSP 管理，业务层只需注册回调

### 定时器回调调度（TIM1msTask）

使用静态回调表替代 `va_arg` 可变参数：

```cpp
static const PulseEntry_t TIM_1ms_Callback_Table[] = {
    {1U,   W25Q64JV_AutoPolling_Callback},
    {1U,   BSP_Key_TIM_1ms_Process_PeriodElapsedCallback},
    {1U,   BMI088_TIM_1ms_Service_PeriodElapsedCallback},
    {1U,   UART_TIM_1ms_Recover_PeriodElapsedCallback},
    {10U,  BSP_WS2812_TIM_10ms_Write_PeriodElapsedCallback},
    {50U,  BSP_Key_TIM_50ms_Process_PeriodElapsedCallback},
    {128U, BMI088_TIM_128ms_Calculate_PeriodElapsedCallback},
};
```

`Pulse_Dispatch()` 根据 `tick_ms % period_ms == 0` 调度，类型安全，无 UB。

> **注意**：`pulse_tick_ms` 为 `uint32_t`，建议对 LCM（当前为 6400）取模，
> 防止约 49.7 天后回绕时所有回调集中触发一次。

---

## 设备驱动

### BMI088 + VQF

完整链路：FIFO 中断触发 → SPI DMA 传输 → `BMI088_Task` 逐帧解算 → 发布 INS 状态。

- 陀螺仪：2000 dps，2 kHz，FIFO 模式，SPI DMA 读取
- 加速度计：±24g，1600 Hz，EXTI 中断触发
- 姿态算法：VQF（Vector Quaternion Filter）替代 EKF，运动/静止零偏估计双启用
- 输出：欧拉角（Yaw-Pitch-Roll）+ 机体系角速度，发布到 `INS_State_Topic`

VQF 关键参数（`System_IMU_Configure()`）：

| 参数 | 值 | 说明 |
|---|---|---|
| `Tau_Accel` | 3.0 s | 加速度修正时间常数 |
| `Bias_Forgetting_Time` | 100.0 s | 零偏遗忘时间 |
| `Rest_Min_Time` | 1.5 s | 静止零偏估计触发时间 |
| `Rest_Threshold_Gyro` | 3.5 °/s | 静止判定角速度门限 |

### DJI 电机（`Class_DJIMotor`）

支持 M2006 / C610、M3508 / C620、GM6020，通过 FDCAN 周期槽收发。

```cpp
Struct_DJIMotor_Init_Config config{};
config.hfdcan     = &hfdcan1;
config.can_id     = 1;
config.motor_type = Enum_DJIMotor_Type::M3508;
config.close_loop = DJI_MOTOR_CURRENT_LOOP | DJI_MOTOR_SPEED_LOOP;
config.outer_loop = DJI_MOTOR_SPEED_LOOP;
config.speed_pid  = Chassis_MakePID(4.5f, 0.05f, 0.0f, 3000.0f, 16000.0f);
motor.Init(config);
```

多电机 `Class_DJIMotor_Group` 保证同一物理帧只发送一次，不同任务不会互相覆盖帧槽。详见 [DJI 电机驱动说明](User_File/Device/Peripheral/Motor/DJImotor/dji_motor.md)。

### 达妙电机（`Class_DMMotor`）

MIT / 位置-速度 / 速度 / 力位混控，模式切换等待回包确认后才更新软件模式；合法反馈直接 Feed Daemon，超过 100 ms 无反馈判定离线。详见 [达妙电机驱动说明](User_File/Device/Peripheral/Motor/DMmotor/dmmotor.md)。

### QD4310

云台专用电机，通过 FDCAN 控制，支持电流模式（`QD4310_SetCurrent`）和内置位置环（`QD4310_SetAngle`）。使能 / 失能需等待 CAN 回包确认。

---

## 内存与 DMA 布局

STM32H7 的 DTCMRAM 不可被 DMA1/DMA2 访问，DMA 缓冲区统一放入 `.dma_buffer` 段：

```ld
.dma_buffer (NOLOAD) :
{
    . = ALIGN(32);
    *(.dma_buffer .dma_buffer*)
    . = ALIGN(32);
} >RAM_D1
```

已放入 `.dma_buffer` 的对象：SPI1~6 管理对象、ADC1~3 管理对象、UART 双缓冲区。

`main.c` 的 MPU 配置将 RAM_D1（`0x24000000`）配置为 non-cacheable，保证 DMA 读写缓冲区不受 D-Cache 一致性问题影响。

FreeRTOS `heap_5` 双区配置（`heap_regions_patched.c`）：

| 区域 | 大小 | 说明 |
|---|---:|---|
| DTCMRAM | 48 KiB | 任务栈、RTOS 对象优先分配 |
| RAM_D1 | 16 KiB | DTCMRAM 不足时自动回退 |

调整 DTCMRAM heap 大小（需小于 65536，按 8 字节对齐）：

```powershell
cmake --preset Debug -DH7_FREERTOS_DTCM_HEAP_SIZE=40960
```

---

## 调试

### SystemView

`System_Init()` 中调用 `SEGGER_SYSVIEW_Conf()`，FreeRTOS patched port 已集成追踪探针。用于观察任务切换时序、中断频率和调度延迟。

### RTT

`SEGGER_RTT_printf` 可在任务中使用。OSPI 回调中的调试 printf 已清除，避免 Flash 操作时污染 RTT 缓冲区。

### Ozone / J-Link

`Debug_IMU_Data`（`User_File/System/debug/sys_debug.h`）是固定 ABI 的调试镜像结构体，字段带单位后缀（`_rad`、`_m_s2`、`_us`），可直接在 Ozone Timeline Data Plot 中观察。修改字段顺序时需同步提升 `Debug_ABI_Version`。

```text
TransportTask → EricTool_USB.Set_Data(3, &Euler_Yaw_rad, &Euler_Pitch_rad, &Euler_Roll_rad)
```

### 栈水位观测

```cpp
UBaseType_t hwm = uxTaskGetStackHighWaterMark(NULL);
size_t free_heap = xPortGetFreeHeapSize();
```

建议在稳定运行一段时间后检查各任务栈余量，`BMI088_Task` 和 `Control_Task` 重点关注。

---

## C / C++ 协作边界

底层 CubeMX / HAL / FreeRTOS 为 C，用户层为 C++。边界通过薄的 `extern "C"` 层隔离：

```
CubeMX C / HAL / FreeRTOS
    ↓  只调用 extern "C" 函数
C ABI 边界（Init.h / callback.h / user_task.h）
    ↓
C++ BSP / Device / Algorithm / Application / Task
```

规则：
- C 可见头文件不使用引用、类、模板、`std::` 类型
- DMA 全局缓冲区必须放入 `.dma_buffer` 段
- FreeRTOS ISR API 只在优先级 ≤ `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`（当前为 5）的中断中调用
- C++ 全局对象构造函数不做硬件操作，硬件初始化统一放进显式 `Init()`
- 禁用 C++ 异常和 RTTI

符号检查（确认 `extern "C"` 未遗漏）：

```powershell
arm-none-eabi-nm build/Debug/H7_BSP.elf | findstr "System_Init Control_Task BMI088_Task HAL_GPIO_EXTI_Callback"
```

出现 `_Z...` 形式说明存在 name mangling，需补 `extern "C"`。

---

## 系统辨识

Yaw 云台已完成双环辨识，数据和脚本存放在 `sysid/`：

| 环 | 模型 | 当前参数 |
|---|---|---|
| 速度环 | `G(s) = 1.051·e^{-30ms·s} / (0.108s+1)` | Kp=0.15, Ki=0.63 |
| 位置环 | 基于速度环模型的串联结构 | Kp=32.00, Ki=0.00 |

位置环实测（Kp=32, Ki=0）：RMSE ≈ 0.374 rad，MAE ≈ 0.149 rad，90% 到达时间 ≈ 0.2 s，P90 超调 ≈ 2.35%。

详见 [系统辨识说明](sysid/README.md)。

---

## 开发约定

- 用户代码放在 `User_File/`，CubeMX 生成文件只在 `USER CODE BEGIN/END` 区域内修改
- 新增用户模块时，同时在根 `CMakeLists.txt` 注册源文件和 include 路径
- Topic 名称统一在 `application_topics.h` 用宏定义，不散落字符串字面量
- 新增 DMA 缓冲区必须加 `__attribute__((section(".dma_buffer")))` 并放入 RAM_D1
- 注册失败必须检查返回值，`Application_RegisterTopics()` 返回 `false` 时 `configASSERT` 会挂起

---

## 待完成 / 已知问题

| 项目 | 说明 |
|---|---|
| `TransportTask` 协议 | 当前只初始化 USB + EricTool 遥测，无上位机命令解析 |
| 遥控器 / 视觉 / 裁判系统接入 | 需通过 `RobotCmd_Set*()` 接入，UART 口已预留 |
| `pulse_tick_ms` 回绕 | 约 49.7 天后集中触发一次，建议对 `lcm(1,10,50,128)=6400` 取模 |
| Pitch 角度 PID | 待 Pitch 辨识后整定，当前使用 QD4310 内置位置环 |
| BMI088 加热器 | 温控 PID 已实现，默认关闭；启用需确认 ADC / Power 链路 |
| UART 具体设备接入 | BSP 已完成，待按实际外设绑定业务回调 |
| `UART4/8/9` | DMA stream 已满，如需使用需改中断 / 阻塞方式 |

---

## 致谢

- [Kylin-6](https://github.com/Kylin-6)：贡献达妙电机驱动（PR #4）和 DJI 电机驱动（PR #5）
- [Meta-Embedded-NG](https://github.com/Meta-Team/Meta-Embedded)（MIT）：应用层边界设计与四舵轮运动学参考
- [basic_framework](https://github.com/NeoZng/basic_framework)（MIT）：动态消息中心和 Daemon 固定注册模型参考
- SEGGER：SystemView + RTT 工具链
- STMicroelectronics：HAL 驱动库与 AN4839 / AN4891 内存布局参考

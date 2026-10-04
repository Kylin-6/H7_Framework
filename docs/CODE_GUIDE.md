# 全项目源码导航与调用约束

本文提供从构建到控制输出的阅读顺序，说明容易被文件名、历史注释或实验记录误导的边界。
具体接口以公开头文件和对应模块文档为准；源码被 CMake 编译，不代表已接入实机业务。
新增或修改行为时更新所属模块文档，避免在本导航重复维护协议参数。

## 1. 构建与硬件绑定

| 入口 | 阅读重点 |
| --- | --- |
| [CMakeLists.txt](../CMakeLists.txt) | `H7_BOARD` 选择源码与应用宏；所有板型产物统一为 `H7_Framework.elf/.map` |
| [CMakePresets.json](../CMakePresets.json)、[CMakeUserPresets.json](../CMakeUserPresets.json) | Debug/Release 与三个板型的独立构建目录 |
| [board_config.h](../User_Config/Board/board_config.h) 与 `*_board_config.cpp` | 电机总线与板载功能资源；不保存机构参数 |
| [board_tasks.h](../User_Config/Board/board_tasks.h) 与 `board_tasks_*.c` | 任务属性、句柄和各板任务集合 |
| [h7_linker.cmake](../User_Config/Linker/h7_linker.cmake)、[h7_memory.ld](../User_Config/Linker/h7_memory.ld) | 替换 CubeMX 默认链接脚本，约束 DMA/普通数据/heap 地址 |
| [heap_regions_patched.c](../User_Config/FreeRTOS_Patch/heap_regions_patched.c) | heap_5 的 DTCM 与 D1 两个区域，地址递增排列并以空项终止 |
| [H7_BSP.ioc](../H7_BSP.ioc)、[Core](../Core) | 外设、DMA、IRQ 与 HAL 初始化；生成文件只在 USER CODE 区集成用户逻辑 |

SingleBoard 的三个应用开关默认关闭；关闭云台时排除其源码与任务调用，也不发布云台反馈。GimbalBoard/ChassisBoard 在配置期固定所属应用开关。
Debug/Release 默认使用 SingleBoard，但已有 CMake 缓存可以改变 `H7_BOARD`，应核对缓存。
烧录选择当前 preset 目录的 ELF；操作入口见 [根 README](../README.md#烧录与观察)。
`Drivers`、`Middlewares`、`SystemView` 和 `USB_DEVICE` 是依赖或生成集成层，不应为了补项目注释改写上游实现。

## 2. 初始化与任务

[main.c](../Core/Src/main.c) 的顺序是 MPU/Cache、HAL 与外设初始化、`System_Init()`、
`osKernelInitialize()`、`MX_FREERTOS_Init()`、`osKernelStart()`。
[System_Init](../User_File/System/Init/Init.cpp) 绑定时间戳与总线，按 BoardConfig 初始化板载设备；
CAN 的 RTOS 资源在内核初始化后由 `MX_FREERTOS_Init()` 建立。

`init_finished` 表示初始化流程结束。必需时基失败产生 FATAL，控制任务停在延时循环；
可选设备失败产生 DEGRADED，其他功能继续运行。读取 [状态和失败位图](../User_File/System/Init/Init.h)
才能判断哪些功能可用，不能只检查 `init_finished`。

| 任务源码 | 创建板型 | 调度与职责 |
| --- | --- | --- |
| [Control_Task.cpp](../User_File/Task/Control_Task.cpp) | SingleBoard | 进入后 High1；TIM4 1 ms 标志唤醒，Input → RobotCmd → Gimbal/Chassis/Shoot |
| [Control_Task_Gimbal.cpp](../User_File/Task/Control_Task_Gimbal.cpp) | GimbalBoard | 进入后 High1；板间 Poll → 输入与命令 → Gimbal/Shoot |
| [Control_Task_Chassis.cpp](../User_File/Task/Control_Task_Chassis.cpp) | ChassisBoard | 进入后 High1；板间 Poll → Chassis |
| [BMI088_Task.cpp](../User_File/Task/BMI088_Task.cpp) | SingleBoard、GimbalBoard | 进入后 High2；SPI 线程标志唤醒，续传并清空 FIFO 样本队列，批次完成后发布 INS |
| [CanTxTask.cpp](../User_File/Task/CanTxTask.cpp) | 全部 | High；每轮先提交离散帧，再提交周期槽，不等待对端执行 |
| [StatusTask.cpp](../User_File/Task/StatusTask.cpp) | 全部 | Low；每 10 tick CheckAll，随后调用已编入电机的设备安全 ServiceAll |
| [TIM_1ms_Task.cpp](../User_File/Task/TIM_1ms_Task.cpp) | SingleBoard、GimbalBoard | Low；静态回调表调度传输恢复、按键、灯效、温控 |
| [TransportTask.cpp](../User_File/Task/TransportTask.cpp) | SingleBoard、GimbalBoard | Normal；USB CDC/EricTool 调试输出，文件名不表示板间 CAN |
| [InsTask.cpp](../User_File/Task/InsTask.cpp)、[StorageTask.cpp](../User_File/Task/StorageTask.cpp) | Ins：单板；Storage：单板和云台板 | 兼容入口，创建后立即退出；没有独立姿态解算或持久化服务 |

当前 RTOS tick 为 1 ms。`osDelayUntil()` 避免执行耗时累积成相对延时漂移，但抢占和执行超时
仍会延迟任务；线程标志重复置同一位会合并，不是积累每个定时中断的 FIFO。
不能用调度轮次推断实际接收时间，业务 freshness 使用统一时间戳。

TIM1msTask 的 128 ms BMI088 回调负责温度请求与加热 PID；姿态解算在 BMI088Task。
按键每 50 ms 采样 GPIO，再每 1 ms 更新边沿状态；没有连续稳定窗口消抖判定。
Pulse 在调用者上下文同步执行回调，tick=0 会触发所有有效项；没有漏轮次补偿。

## 3. 控制、状态与通信

具体业务看 [Application](../User_File/Application/README.md)，消息语义看
[Message Center](../User_File/System/MessageCenter/README.md)，板间格式看
[Transport](../User_File/System/Transport/README.md)。

- InputState 的 Setter 与 RobotCmd 的缓存更新由同一 ControlTask 调用；UART ISR 只保存设备帧。
- RobotCmd 拥有连续命令发布；应用拥有对应 Device 的控制目标，避免多处同时修改电机输出。
- Gimbal 的 UpdateTarget 合并姿态捕获条件，恢复后 IMU 等待新序号；Shoot 在模式处理后统一选择拨弹外环和目标；RobotCmd 失联和切源共用射击事件清理。
- Topic 的数据和元信息受短 PRIMASK 临界区保护；需要关联时调用 ReadWithMeta。
- Subscriber 自己的已读序号没有额外同步，同一订阅实例由单一上下文持有。
- EventQueue 满时拒绝新事件，Push 成功只表示逻辑动作已接受，不代表机构执行完成。
- BoardTransport 的 RX ISR 保存帧与接收时刻，控制任务解码后 PublishAt；处理延迟不会延长命令时效。
- [Daemon](../User_File/System/Daemon/README.md) 判断合法数据流是否持续；电机在线判断统一读取 Daemon；Online 不替代设备 Ready 或业务 Topic 时效。
- 电机自行处理基础掉线安全输出，StatusTask 的 ServiceAll 独立维护保护；App 只表达功能许可、生成目标和处理机构恢复，WS2812 不检测设备掉线。

[时间戳](../User_File/System/Timestamp/sys_timestamp.h) 使用 TIM5 的 1 MHz 计数与软件溢出扩展。
ARR 为 `3600000000-1`，每 3600 s 更新一次；Init 只绑定句柄。延时 helper 为忙等待，不让出 CPU。
[调试数据](../User_File/System/debug/sys_debug.h) 是外部采样 ABI；读取者需在完整数据复制前后复核 sequence，并核对 sequence/sequence_end
均为同一偶数，不能把 volatile 多字段读取当成一致快照。USB 遥测读取部分字段不等同于完整 ABI 快照。
[Flash 布局](../User_File/System/Storage/sys_flash_layout.h) 预留末尾两个扇区；地址声明不等于已实现零偏持久化。

## 4. BSP 与 Device

BSP 入口、DMA 布局和所有权由 [BSP 指南](../User_File/Middleware/BSP/README.md) 维护。
CAN/UART/SPI 回调运行环境应按实际入口核对，不能因为函数名含 `Callback` 就假定它在 ISR 中。
`TIM_*_PeriodElapsedCallback` 在多个模块中实际由任务调用。

| 设备 | 源码/文档入口 | 调用边界 |
| --- | --- | --- |
| DJI | [dji_motor.md](../User_File/Device/Peripheral/Motor/DJImotor/dji_motor.md) | Application 使用输出轴 rad/rad/s；CAN 原始电流和反馈 RPM 在驱动边界转换 |
| 达妙 | [dmmotor.md](../User_File/Device/Peripheral/Motor/DMmotor/dmmotor.md) | RequestEnabled 管输出许可；ServiceAll 维护协议状态；提交成功不等于反馈确认 |
| S.BUS / DBUS | [Remote README](../User_File/Device/Peripheral/Remote/README.md) | S.BUS 已接 UART5；DBUS 的 chunk 解析不是有界流式拼帧器 |
| Referee / VTM | [Referee README](../User_File/Device/Peripheral/Referee/README.md) | 默认未绑定业务；在线不证明某个业务字段有效，115 ms 发送延时不能进入控制周期 |
| BMI088 | [bsp_bmi088.h](../User_File/Device/Onboard/BMI088/bsp_bmi088.h)、[IMU](../User_File/System/IMU/README.md) | FIFO/SPI 接收与任务解算分离；有限的有效姿态才发布 INS/Feed |
| W25Q64JV | [bsp_w25q64jv.h](../User_File/Device/Onboard/W25Q64JV/bsp_w25q64jv.h) | 显式 Init 与 Busy/超时状态；不把 Flash 完成回调当成后台持久化服务 |
| Power / Key / WS2812 / Buzzer | [Onboard](../User_File/Device/Onboard) | 电源、按键、灯效、蜂鸣器属于 Device，物理句柄由板级集成提供 |
| EricTool | [dvc_erictool.h](../User_File/Device/Peripheral/EricTool/dvc_erictool.h) | 调试字典与周期输出；回调长度和 UART 发送状态不能忽略 |

UART DMA 发送先复制至专用缓冲，无 TX DMA 的端口则走阻塞 HAL 路径。BSP 不负责协议拼帧。
ADC_BUFFER_SIZE 是 uint16_t 元素个数；ADC_Init 的 Sample_Number 也是元素数，调用方须守住容量，
循环模式由 CubeMX 配置决定。volatile 不能替代同步，也不能解决 DMA Cache 一致性。

## 5. 算法阅读入口

所有算法由调用者控制输入时间、采样频率与执行上下文。函数名包含 TIM 不会自动创建定时器或线程。
模型输入单位必须匹配设备实际输出，特别是协议电流原始值不能与 N·m 混用。

| 组件目录 | 阅读重点 |
| --- | --- |
| [PID](../User_File/Middleware/Algorithm/PID) | D_T 为 s，滤波截止频率为 Hz；死区不保证总输出为零，Init 更新参数不等于清空历史 |
| [SMC](../User_File/Middleware/Algorithm/SMC)、[DOB](../User_File/Middleware/Algorithm/DOB) | SMC 用同一时刻状态和导数；DOB 的 y[k] 对齐上一周期限幅后的实际 u[k-1] |
| [Trajectory](../User_File/Middleware/Algorithm/Trajectory) | 位置量纲 U 对应 U/s、U/s²、U/s³；改目标从规划状态接续，机械位置边界由应用限制 |
| [Fuzzy](../User_File/Middleware/Algorithm/Fuzzy) | 有序节点与完整规则表由调用方持有，不提供预设机构控制规则 |
| [Filter](../User_File/Middleware/Algorithm/Filter) | FIR/IIR/OneEuro/Polynomial 依赖固定新采样；缺测不能靠重复输入旧值掩盖 |
| [Kalman](../User_File/Middleware/Algorithm/Filter/Kalman)、[EKF](../User_File/Middleware/Algorithm/Filter/EKF) | 模型与协方差由使用方配置；Kalman 缺测时只预测，是否有有效测量由调用者决定 |
| [VQF](../User_File/Middleware/Algorithm/Filter/VQF) | 整机参数入口是 sys_imu.cpp；采样、坐标系和单位与 BMI088 链路配合 |
| [Matrix](../User_File/Middleware/Algorithm/Matrix)、[Quaternion](../User_File/Middleware/Algorithm/Quaternion)、[Complex](../User_File/Middleware/Algorithm/Complex)、[Basic](../User_File/Middleware/Algorithm/Basic) | 数学组件；维数、坐标约定和输入数值范围由调用方保证 |
| [Slope](../User_File/Middleware/Algorithm/Slope)、[FSM](../User_File/Middleware/Algorithm/FSM)、[Pulse](../User_File/Middleware/Algorithm/Pulse) | 变化限幅、状态存储和静态周期分发，不替代应用状态机或实时调度器 |
| [Queue](../User_File/Middleware/Algorithm/Queue)、[CRC](../User_File/Middleware/Algorithm/CRC) | Class_Queue 无并发保护且满时静默拒绝；业务事件用 EventQueue，CRC 按所属协议选择 |

## 6. 实验与验证资料

[sysid](../sysid/README.md) 的旧实验模型和参数属于历史控制器，不能直接作为当前达妙 MIT 云台参数。
生成激励数据文件也不代表固件已消费该序列；采集前核对日志格式、单位和实际激励路径。
[架构图](../README.md#维护架构图) 的 JSON/HTML/SVG 共用现有工具链，源码约束改变时更新图源再生成。

纯注释与文档修改应核对本地链接、注释与源码事实，并确认不改变可执行代码。构建覆盖受影响板型；
没有硬件时，不能声称 DMA、CAN、
电机闭环或最坏执行时间已验证。历史更新记录记录当时事实，不改写成当前行为。

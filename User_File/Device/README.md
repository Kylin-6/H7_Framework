# Device 函数使用指南

本文按设备说明公开函数的用途、调用顺序和应用接入方式。范围包含 `Onboard` 与 `Peripheral`；
板载设备中名为 `bsp_*` 的文件也属于本目录的 Device，不是 Middleware/BSP 的总线接口。
具体参数声明以各设备头文件为准，协议细节通过下方链接查阅。

## 使用顺序与所有权

1. CubeMX 初始化外设，System/Task 按依赖顺序初始化总线、设备与 RTOS 服务。
2. 应用在 Init 中注册自己拥有的设备，不在构造函数中访问硬件。
3. 周期更新读取设备快照、判断许可并提交目标；返回成功不代表设备执行成功。
4. 底层回调由 BSP/System 分发，周期服务由现有任务调用；不要另建重复的 HAL 回调或服务任务。

电机实例保持静态生命周期，同一总线设备编号和发送报文必须有唯一所有者。
Application 使用 Device 接口，不直接解析 CAN/UART 或管理 DMA。
板型选择见根 CMake 和 BoardConfig：目录中存在驱动不代表所有板型都编译或初始化该设备。
移植约束见 [Application](../Application/README.md)、[BSP](../Middleware/BSP/README.md) 和 [Message Center](../System/MessageCenter/README.md)。

## 设备导航

| 设备 | 头文件 | 当前接入方式 |
| --- | --- | --- |
| 达妙电机 | [dmmotor.h](Peripheral/Motor/DMmotor/dmmotor.h) | Gimbal 拥有设备；StatusTask 维护 ServiceAll |
| DJI 电机 | [dji_motor.h](Peripheral/Motor/DJImotor/dji_motor.h) | Chassis/Shoot 初始化电机与发送组 |
| QD4310 | [QD4310.h](Peripheral/Motor/QDrive/QD4310.h) | 提供驱动接口，需业务自行接入 |
| S.BUS / DJI 遥控 | [sbus.h](Peripheral/Remote/sbus.h)、[remote_control.h](Peripheral/Remote/remote_control.h) | 当前 Input 绑定 UART5 S.BUS |
| 裁判 / VTM / UI | [Referee 指南](Peripheral/Referee/README.md) | 当前 System_Init 不自动绑定，业务数据尚未接入 Shoot/Chassis |
| EricTool | [dvc_erictool.h](Peripheral/EricTool/dvc_erictool.h) | UART/USB 调试与遥测 |
| BMI088 | [bsp_bmi088.h](Onboard/BMI088/bsp_bmi088.h) | System 初始化、BMI088_Task 解算，应用读取 INS Topic |
| W25Q64JV | [bsp_w25q64jv.h](Onboard/W25Q64JV/bsp_w25q64jv.h) | OSPI2 外部 Flash 与存储任务 |
| 电源 / 按键 / 蜂鸣器 / RGB | [Power](Onboard/Power/bsp_power.h)、[Key](Onboard/Key/bsp_key.h)、[Buzzer](Onboard/Buzzer/bsp_buzzer.h)、[WS2812](Onboard/WS2812/bsp_ws2812.h) | System 初始化，现有周期任务维护 |

## 达妙 Class_DMMotor

完整协议与状态语义见 [达妙驱动指南](Peripheral/Motor/DMmotor/dmmotor.md)。

| 函数 | 怎么使用 |
| --- | --- |
| Init(bus,id,master_id,mode,reverse,pmax,vmax,tmax) | 注册 CAN 回调和在线守护；ID、反馈 ID、模式及量程须与电机端一致；检查 bool |
| RequestEnabled(enabled) | 表达期望使能；重复同请求不会重复执行收发，失能先提交安全目标；返回不表示实际使能 |
| SetMIT(position,velocity,kp,kd,torque) | MIT 模式，rad、rad/s、N·m；增益按电机端协议，应用先限幅 |
| SetPositionSpeed(position,velocity) | 位置速度模式的 rad 与 rad/s 目标 |
| SetSpeed(speed) | 速度模式目标 rad/s，速度闭环由电机端执行 |
| SetForcePosition(position,velocity_limit,current_limit_ratio) | 力位混合模式，位置/速度为 SI，电流限制为 0～1 协议比例 |
| SetTorque(torque) | 使用 MIT 转矩输出路径，输入 N·m |
| ClearError / SetZeroPosition / SetMode | 离散操作，检查提交结果；置零/模式改变是实际设备操作，不能每周期重复调用 |
| GetFeedbackSnapshot | 一次读取运动反馈、online、requested_enabled、actual_enabled、fault、ready |
| IsOnline / IsDataValid | 最新反馈是否有效；IsDataValid 当前等价于 IsOnline |
| IsEnabled | 最新合法反馈是否报告协议已使能 |
| IsHealthy | 请求使能、在线且协议已使能 |
| GetDaemon | 只读守护状态，用于诊断，不替代反馈快照 |
| ServiceAll | 现有 100 Hz StatusTask 调用，补交使能/失能与安全目标；业务不另写协议重试 |

Init 的模式参数不会自动切换电机端模式，SPEED 等模式必须预先配置正确。
优先使用快照，不直接跨任务读取公开 feedback 的多个字段。

<details>
<summary>例程：初始化和速度目标提交（两个函数分别在启动、周期调用）</summary>

```cpp
#include "dmmotor.h"
static Class_DMMotor motor;
static bool registered = false;

bool MotorExample_Init(FDCAN_HandleTypeDef *bus)
{
    registered = motor.Init(bus, 1U, 0x60U, Enum_DMMotor_Mode::SPEED,
                            false, 12.5f, 30.0f, 10.0f); // 示例量程必须匹配电机端。
    return registered;
}

bool MotorExample_Update(bool allowed, float speed_rad_s)
{
    if (!registered) // 注册失败时不运行控制路径。
    {
        return false;
    }
    if (!allowed) // 应用许可关闭，撤销主动输出并交给驱动维护安全目标。
    {
        return motor.RequestEnabled(false);
    }
    (void) motor.RequestEnabled(true); // 期望使能不等于反馈确认。
    Struct_DMMotor_Snapshot snapshot = motor.GetFeedbackSnapshot();
    if (!snapshot.ready) // 在线、实际使能等就绪条件尚未满足。
    {
        return false;
    }
    return motor.SetSpeed(speed_rad_s); // 返回只表示软件周期槽接受目标。
}
```

调用方保证速度有限且按机构限幅；保留现有 ServiceAll 调度。更完整的多电机故障策略见应用文档。

</details>

## DJI Class_DJIMotor 与发送组

详细闭环配置见 [DJI 驱动指南](Peripheral/Motor/DJImotor/dji_motor.md)，完整应用见 [Shoot](../Application/Shoot/README.md)。

| 电机函数 | 怎么使用 |
| --- | --- |
| Init(config) | 指定总线、型号、编号、减速比、环配置和 PID；全部成员 Init 后才绑定发送组 |
| SetRef / SetRef_Degree | 设置目标；角度环 rad、速度环 rad/s，Degree 版本无条件转弧度，不能用于协议电流值 |
| Control() | 计算单电机指令，不负责发送整组报文 |
| RequestEnabled / Enable / Disable | 本地输出许可；与达妙反馈确认使能的语义不同，失能清零指令 |
| Set_Outer_Loop | 在已配置的级联环中选择目标入口，例如拨弹角度/速度模式 |
| Set_Feedback_Source | 指定角度/速度环反馈来源；外部 float 指针需长期有效且与环单位一致 |
| GetMotionSnapshot | 一致读取输出轴累计角 rad、角速度 rad/s 和在线/许可/ready |
| Get_Last_Feedback_Timestamp_Us | 安全读取最近反馈微秒时间戳，避免 32 位 MCU 直接读取撕裂 |
| IsOnline / IsEnabled / IsDataValid / IsHealthy | 分别检查反馈时效、本地许可、可用数据与综合健康 |
| GetDaemon | 只读链路诊断 |

| 发送组函数 | 怎么使用 |
| --- | --- |
| Init(motor1,...motor4) | 绑定同一总线、同一物理发送 ID 的全部已注册成员，目标顺序对应参数顺序 |
| SetRef / SetRef_Degree | 批量设置目标，不计算、不提交报文 |
| Update(ref1,...) | 设置目标并计算，仍需 Send |
| Control() 无参数 | 使用已保存目标计算，仍需 Send |
| Control(ref1,...) / Control_Degree(ref1,...) | 设置、计算并提交；bool 同时要求提交成功且成员 ready |
| Send | 提交共享组报文，不确认硬件发送或电机执行 |
| RequestEnabled / Enable / Disable | 批量管理成员许可 |

不能只调用单电机 Control 而漏掉组 Send；不能让不同逻辑组占用同一物理组报文。
公开 feedback 和 PID 对象不是整体原子快照；应用读取运动状态使用 GetMotionSnapshot。

## QD4310

此驱动的速度边界仍是 rpm，电流 A、角度 rad；应用中的 rad/s 目标需先乘 60/(2π) 转为 rpm。

| 函数 | 用途 |
| --- | --- |
| QD4310_Init | 绑定静态 QD4310_t、编号和总线；无返回值，检查 initialized |
| QD4310_Enable / Disable | 提交离散使能/失能，检查 bool，不代表执行确认 |
| SetAngle / SetStepAngle / SetZeroAngle | 绝对角、步进角、机械置零；前两者输入 rad |
| SetSpeed / SetLowSpeed | rpm 速度目标，头文件范围 ±1000 rpm |
| SetCurrent | A 电流目标，头文件范围 ±10 A |
| SendCommand | 底层命令枚举和 int16 编码值接口，优先使用物理量封装 |
| Update | 解析 8 字节反馈，由接收路径调用，业务不重复调用 |
| IsOnline / IsEnabled / IsDataValid / IsHealthy | 最近 100 ms 反馈、反馈使能、数据可用及综合状态 |

QD4310_t 由 CAN ISR 更新，没有内置多字段快照锁，任务读取 speed/angle/current 必须自行做一致性保护。

## 遥控 S.BUS 与 DJI 遥控

接线和协议见 [Remote 指南](Peripheral/Remote/README.md)，业务转换见 [Input](../Application/Input/README.md)。

| 函数 | 用途 |
| --- | --- |
| SBUS_Init(huart) | 绑定 UART 接收；当前 RemoteInput 已绑定 UART5，不要重复占用该端口 |
| SBUS_ReadLatest(&frame) | 取得完整帧快照，重复调用可读取同一序号，不表示新帧 |
| SBUS_GetDiagnostics | 读取接收、重同步、合法帧和失控标志统计 |
| SBUS_IsEnabled / Online / DataValid / Healthy | Enabled 为完成绑定，Online/DataValid 为守护在线，Healthy 还排除 frame_lost/failsafe |
| SBUS_RxCallback | 交给 BSP 的流式 chunk 解析入口，业务不另注册竞争回调 |
| RemoteControlInit | DJI 遥控协议的独立入口，返回静态 RC_ctrl_t 数据指针 |
| RemoteControlIsOnline / Enabled / DataValid / Healthy | 对应 DJI 遥控链路状态；不等于已接入 RobotCmd 仲裁 |

S.BUS 通道已减 1024，索引 0 对应 CH1；sequence/timestamp_ms 用于判断新帧与年龄。
Device 在线门限 100 ms 与 Input 的 50 ms 安全门限不同。应用使用失控位与更严格的策略，不能只看 Online。
DJI 遥控反馈指针不是本工程 InputState 的同步快照，接入时须处理 ISR/任务一致性。

## 裁判系统、VTM、UI 与 CRC

| 函数 | 用途与限制 |
| --- | --- |
| RefereeInit / VTMInit | 显式绑定已配置 RX DMA 的 UART，返回静态数据指针；失败返回 NULL |
| RefereeReceiveData / VTMReceiveData | 数据解析入口；Referee 支持跨 chunk 半帧，VTM 当前不提供相同重组保证 |
| RefereeSend / VTMSend | 发送交互数据，成功提交后均等待 115 ms，不能放入 ISR 或 1 kHz 控制路径 |
| RefereeIs* / VTMIs* | Enabled 为绑定；Online 由合法帧维护，DataValid 不保证每个字段都新鲜 |
| UILineDraw / UIRectangleDraw / UICircleDraw / UIOvalDraw / UIArcDraw | 填写图元，名称是固定 3 字节，坐标/层/颜色按协议范围 |
| UIFloatDraw / UIIntDraw / UICharDraw | 填写数值或文字图元，参数布局见 referee_UI_26.h |
| UIGraphRefresh / UICharRefresh / UIDelete | 打包发送图元或删除层，调用前建立本机/客户端 ID |
| Get_CRC8/16_Check_Sum | 计算校验和，传协议初始值 |
| Verify_CRC8/16_Check_Sum | 校验完整数据，检查返回值 |
| Append_CRC8/16_Check_Sum | 将校验和写入消息尾部，缓冲必须含校验字段空间 |
| RefereeDaemon*/VTMDaemon* | C 解析器使用的注册/Feed/Online 薄接口，应用优先用 RefereeIs*/VTMIs* |

RefereeSend 成功提交后等待 115 ms，仅供允许阻塞的低优先级任务。Referee/VTM 静态反馈由中断更新，
直接跨任务读取多个字段需建立业务快照；链路 Online 不等于热量/射速等字段已经更新。
当前数据未自动进入 Shoot/Chassis，VTM 也未自动提交 InputState，图传业务载荷尚未完整实现。

## BMI088 姿态与传感器

System 已配置 VQF 并初始化 BSP_BMI088，BMI088_Task 负责解算；应用优先通过 INS_State_Topic 读取一致且有时效的状态。

| 函数组 | 用途 |
| --- | --- |
| Set_VQF_Config / Init / Is_Initialized | 在 Init 前设置配置，显式启动传感器，检查结果；不在应用中重复初始化 |
| Calculate | 姿态解算任务调用，不在 EXTI/SPI ISR 执行完整运算 |
| EXTI_Flag_Callback / SPI_RxCpltCallback | 采集状态推进，由既有 HAL/BSP 回调接入 |
| BMI088_Service_Transfer / TIM_1ms_Service_PeriodElapsedCallback | 传输推进、超时与恢复服务，保留现有任务调用 |
| TIM_128ms_Calculate_PeriodElapsedCallback | 慢周期维护入口，依当前任务调度使用 |
| Get_Original_Accel/Gyro、Get_Fixed_Corrected_Gyro/Offset | 原始/固定校正后的数据与偏置，单位参见对应头文件及配置 |
| Get_Accel_Body/Gyro_Body、Get_Accel/Gyro | 机体/驱动输出向量；不得忽略坐标约定直接替换 INS 字段 |
| Get_Euler_Angle / Rotation_Matrix / Axis_Angle / Quaternion | 当前姿态表示，四元数/矩阵用途不同，应用控制接口使用 rad |
| Get_Accel_Norm / D_T / Calculating_Time | 模长、采样周期 s、计算耗时；时间量按头文件/计时实现核对 |
| Get_*Counter、Get_*Result、Get_*Reason | FIFO、异常采样、SPI 超时/恢复、VQF 重置等诊断 |
| Get_VQF_* | 陀螺偏置、静止检测及滤波诊断，保持解算任务所有权 |
| Get_Atomic_Copy | 驱动 getter 的短临界区复制辅助，不保证多个 getter 来自同一帧 |

Accel 子驱动：Init、SPI_Request_Accel/Temperature、SPI_RxCpltCallback 为采集链接口；
Get_Raw_Accel/Get_Valid_Flag 读取采样与有效性；温度 getter 提供温度、年龄、异常和加热 PWM。
Set_Target_Temperature/Set_Heater_Enable 设置温控策略，TIM_128ms_Heater_PID_PeriodElapsedCallback 维护加热。
Gyro 子驱动：Init/Start_FIFO_Acquisition 开始 FIFO；Notify_FIFO_Interrupt 记录中断时刻；
SPI_Request_Gyro/SPI_RxCallback 推进读 FIFO；Pop_Sample 从软件队列取一条样本；其余 Get_* 为采样或队列诊断。
这些子驱动由 BMI088 采集链拥有，应用不并发抢占同一路 SPI。

<details>
<summary>例程：应用读取 INS，不直接读取多个传感器 getter</summary>

```cpp
#include "message_center.h"
INS_State state;
bool fresh = MessageCenter::INS_State_Topic.ReadFresh(state, 10000U); // 10 ms，单位 us。
if (!fresh) // 没有足够新鲜的姿态，应用应进入自己的安全/等待路径。
{
    return;
}
float yaw_rad = state.yaw_rad; // 控制层使用 SI 角度，安装坐标由传感器集成层处理。
```

</details>

## W25Q64JV Flash

| 函数 | 用途与限制 |
| --- | --- |
| Init(mode) / Is_Initialized / Is_Ready | 初始化、检查启动与空闲；默认 Normal 模式，OSPI2 已由 System 绑定 |
| Enable_Quad_Mode | 配置 Quad 位，属于设备初始化流程 |
| Get_Buffer(address,length) | 异步读取到管理对象 Rx_Buffer，完成回调后取数据，length 为字节数 |
| Set_Write_Enable | 提交写使能；编程/擦除前按流程调用并等待完成 |
| Set_Buffer(buffer,address,length) | 异步页编程，先写使能并核对页边界；提交成功不是写完 |
| Set_Sector_Erased | 4 KiB 扇区擦除，检查 bool 并等待 Ready |
| Set_Bolck_Erased_32K / 64K、Set_Chip_Erased | 块/整片擦除，接口沿用拼写 Bolck，实际会删除内容 |
| Read_Data / Write_Data | 同步分块读写，内部 osDelay(1) 等待，放在允许等待的存储任务 |
| Get_Auto_Polling_Error_Count | 检查自动轮询失败，Ready 不等于上次操作成功 |
| OSPI_StatusMatchCallback / RxCallback / TxCallback | 由现有 OSPI 回调分发，业务不手动冒充完成回调 |
| TIM_1ms_AutoPollingTimeout_PeriodElapsedCallback | 现有任务维护轮询超时，保留调用 |

地址单位 byte，Flash 8 MiB、页 256 byte，DMA 缓冲 512 byte。Write_Data 自动处理页边界与写使能，
不自动擦除旧内容；Read_Data/Write_Data 不用于 ISR 或 1 kHz 控制任务。
Get_Buffer 的结果在 OSPI 管理缓冲中，下一次传输会复用，消费后应复制到自己的对象。

## 电源、按键、蜂鸣器与 RGB

| 类/函数 | 怎么使用 |
| --- | --- |
| BSP_Power.Init(dc24_0,dc24_1,dc5) | 初始化 ADC 与三路供电输出，默认输出关闭；System 已按板型调用 |
| Get_Power_Voltage | 从采样估算电压 V，需先成功建立 ADC 路径 |
| Set_DC24_0 / Set_DC24_1 / Set_DC5 | 修改对应供电 GPIO，属于实际供电操作 |
| BSP_Key.Init / Get_Key_Status | 初始化板载键并读取 FREE/PRESSED 与按下/松开边沿状态 |
| Key TIM_1ms_Process / TIM_50ms_Read_PeriodElapsedCallback | 现有任务根据慢采样更新边沿，当前不是完整连续去抖算法 |
| BSP_Buzzer.Init(frequency,loudness) | 启动 PWM，默认 4000 Hz、响度 0 |
| Set_Frequency / Set_Loudness / Set_Sound | Hz 与归一化响度，调用方使用合理频率和 0～1 响度，0 静音 |
| BSP_WS2812.Init / Set_Red/Green/Blue / Set_RGB | 初始颜色或 0～255 RGB 分量 |
| Set_Color(color,brightness) | 使用 WS2812_COLOR_* 常量和 0～1 亮度 |
| TIM_10ms_Write_PeriodElapsedCallback | 现有任务发送缓存颜色，不要从多个任务并发抢发 SPI |

Key/WS2812 的 `BSP_*_TIM_*` C 包装函数用于现有任务连接，不是另一套设备实例。
按键边沿不是事件 FIFO，若消费频率过低可能漏过边沿，业务需按任务节拍读取。

<details>
<summary>例程：修改指示颜色和蜂鸣器（设备已由 System 初始化）</summary>

```cpp
#include "bsp_ws2812.h"
#include "bsp_buzzer.h"
void ShowReady()
{
    BSP_WS2812.Set_Color(WS2812_COLOR_GREEN, 0.2f); // 修改缓存，现有 10 ms 服务负责写出。
    BSP_Buzzer.Set_Sound(4000.0f, 0.1f); // Hz 与响度；结束提示时由业务调用 Set_Loudness(0)。
}
```

</details>

## EricTool 调试

| 函数 | 用途与限制 |
| --- | --- |
| UART/USB Init | 绑定端口/变量字典及帧尾；字典与变量生命周期须覆盖调试对象 |
| Set_Data(Number,...) | 登记待绘图变量地址，不是值；最多 24 个通道，当前变参实现按 int 取地址，仅适合既有 32 位 MCU 用法 |
| Get_Variable_Index / Value | 获取下行 variable:value# 的解析结果，索引 -1 表示未匹配；业务自己校验并应用 |
| UART_RxCpltCallback / USB_RxCallback | 输入 chunk 的解析入口，按现有回调绑定调用 |
| TIM_1ms_Write_PeriodElapsedCallback | 生成/提交 justfloat 帧；UART 返回提交状态，USB 版本没有返回值 |
| EricTool_Send_Telemetry | 当前工程统一遥测入口，优先沿用既有调度而非新增并行发送者 |

Set_Data 保存地址，不能传临时栈变量或直接传 float 值；不要把 MCU 地址变参用法直接照搬到 64 位主机。
控制任务不等待 UART/USB Busy，遥测发送应由已有 Transport 调度承担。

## 移植与验证

公开函数声明见对应头文件；寄存器头、协议结构体和 CRC/UI 编码辅助由所属驱动使用。
新增驱动在根 CMake 显式登记源码与 include，并确定唯一设备所有者、反馈同步方式和回调生命周期。
验证应覆盖初始化失败、参数/范围、提交 Busy/失败、设备离线、安全目标和恢复，实机核对方向、
单位、减速比、接线、DMA 可达性及 CAN/UART 负载。文档例程不替代实际机构参数标定。

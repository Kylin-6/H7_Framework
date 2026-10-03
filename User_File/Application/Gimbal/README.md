# 双达妙云台

云台由统一 ControlTask 以 1 kHz 调度，两轴均使用现有 `Class_DMMotor` 和 MIT 模式。
QD4310 驱动仍作为独立设备保留，云台不再依赖它。SingleBoard 默认
`H7_APP_GIMBAL=OFF`；GimbalBoard 构建固定启用云台硬件路径。

## 移植时先看哪些文件

现有云台可以直接作为 Application 示例使用。只换接线、电机编号和机构参数时，
先改配置；更换控制算法时，主要改 `Gimbal.cpp` 内部实现。下面的消息和任务入口是
云台与框架之间的接口，重新开发时也需要对接。

| 文件 | 移植或重新开发时的职责 |
| --- | --- |
| [Gimbal.h](Gimbal.h) | 保留对外初始化、周期更新、状态读取入口 |
| [Gimbal_Config.h](Gimbal_Config.h) | 电机参数、机构限位、方向、IMU 轴和控制增益 |
| [Gimbal.cpp](Gimbal.cpp) | 私有设备对象、控制器、目标管理、停机和反馈发布 |
| [SingleBoard 控制任务](../../Task/Control_Task.cpp)、[GimbalBoard 控制任务](../../Task/Control_Task_Gimbal.cpp) | 启动时初始化一次，每 1 ms 调用更新；选择所用板型对应的文件 |
| [RobotCmd](../RobotCmd/RobotCmd.cpp) 与 [Input 仲裁接口](../Input/source_arbitration.h) | 组织遥控、主机、视觉等输入，统一生成云台命令 |
| [消息类型](../../System/MessageCenter/message_types.h)、[消息通道](../../System/MessageCenter/message_center.h) | 云台命令、INS 状态、反馈的数据契约 |
| [板型与 Transport](../../System/Transport/README.md) | 板型硬件绑定与双板通信，接线变更从 BoardConfig 核对 |
| [根 CMakeLists.txt](../../../CMakeLists.txt) | 选择板型、注册新增源码和 include 路径 |

### 必须对接的公开函数

| 函数 | 谁调用、何时调用 | 云台实现需要完成什么 |
| --- | --- | --- |
| `bool Gimbal_Init(const Struct_Gimbal_Config& config)` | 所选 ControlTask 启动阶段，只调用一次；省略参数时使用 `Gimbal_Default_Config()` | 校验并复制配置、注册本模块拥有的设备、初始化控制器；成功返回 true，失败返回 false |
| `void Gimbal_Update()` | 同一个 ControlTask 每 1 ms 调用一次，在 `RobotCmd_Update()` 之后 | 读取输入快照、判断运行许可、处理目标、计算并提交设备输出、按分频发布反馈 |
| `Enum_Gimbal_Status Gimbal_GetStatus()` | 同任务上下文按需读取，仅在 `GIMBAL=1` 时存在 | 提供只读状态；读取本身不执行控制、重试或恢复 |

`Gimbal_Update()` 在 `GIMBAL=0` 时仍存在：读取 INS 并以 100 Hz 发布姿态反馈，
不创建电机控制路径。`Gimbal_Init()` 和状态枚举只在 `GIMBAL=1` 时编译。
因此新实现也应保留关闭硬件路径时可编译的更新入口。

现有两份 ControlTask 都调用了初始化，但没有处理其返回值；当前应用通过内部
`initialized` 阻止初始化失败后的正常控制。移植时若需要启动故障记录，可在已有
初始化调用位置处理返回值，不要另加一次初始化或在周期内重复注册电机。

### 框架已经安排好的调用顺序

SingleBoard 每周期执行 `RemoteInput_Update()` → `RobotCmd_Update()` →
`Gimbal_Update()` → `Chassis_Update()` → `Shoot_Update()`。
GimbalBoard 先执行 `BoardTransport_Poll()`，再更新输入、RobotCmd、云台和发射。
ChassisBoard 不编译云台 Application。

ControlTask 等待 1 ms 线程标志，控制算法运行在任务上下文。云台无需新建任务，
也不要把 `Gimbal_Update()` 放进 CAN 接收中断。CAN 报文解析、发送槽和 DMA 归 BSP/Device；
现有 [StatusTask](../../Task/StatusTask.cpp) 每 10 ms 调用达妙 `ServiceAll()`，
维护协议提交与设备状态。只修改控制算法时保留这些调度。

## 云台如何收命令、读传感器、发反馈

云台通过已有静态 Topic 对接框架，不需要注册字符串通道或再建一套消息总线。

| 通道 | 写入者 → 读取者 | 使用方式 |
| --- | --- | --- |
| `Gimbal_Command_Topic` | RobotCmd → Gimbal | `ReadWithMeta()` 取得命令、有效标志及发布序号；连续目标只保留最新值 |
| `INS_State_Topic` | System IMU → Gimbal | `ReadFresh(ins, 10000U)` 检查最多 10 ms 的状态；角度 rad、角速度 rad/s |
| `Gimbal_Feedback_Topic` | Gimbal → RobotCmd/其他消费者 | 每 10 个控制周期 `Publish()`；反馈姿态来自 INS，不是电机编码器位置 |

### 命令字段与模式

`GimbalCmd` 包含 `mode`、`yaw_angle_rad`、`pitch_angle_rad`、
`yaw_speed_rad_s`、`pitch_speed_rad_s`。角度目标使用 INS 姿态坐标；速度字段是
IMU 模式的角速度前馈。电机编码器零位与 INS 零位不是同一个概念。

| 模式 | 当前行为 |
| --- | --- |
| `DISABLED` | 请求两轴失能，优先于其他控制条件 |
| `LOCK` | 进入模式或恢复就绪时捕获当前姿态，保持捕获值，忽略命令中的角度和速度字段 |
| `IMU` | 使用新发布序号对应的姿态目标与速度前馈；恢复时先捕获姿态，恢复前的目标不重放 |

遥控映射或自瞄目标应接入 Input/RobotCmd 的现有命令组织路径。不要从云台内部
发布自己的 `Gimbal_Command_Topic`，也不要让其他任务同时直接控制这两个电机。
`RobotCmd_SetGimbal(command)` 更新 RobotCmd 的待发布值，未 armed 时会忽略请求；
实际发布发生在 `RobotCmd_Update()`，其输入仲裁也可能更新待发布值。
因此它不是任意任务随时调用都能独占目标的接口。新增输入来源应先处理仲裁和许可，
不能只在外部循环里调用 setter。

当前 RobotCmd 按目标变化发布云台命令，云台不对命令年龄做超时判断。
重新开发时若增加命令超时，必须一起设计发布刷新周期，否则静止目标也会被误判失联。
恢复后的相同数值目标若没有再次发布，也不会得到新序号；需要在命令所有者处明确
重新发布的时机，而不是由云台修改 Topic 的序号。

### 读取和发布示例

以下是 `Gimbal_Update()` 内部接口用法，省略了控制、使能和恢复逻辑，不能作为完整控制器替换：

```cpp
const auto message = MessageCenter::Gimbal_Command_Topic.ReadWithMeta(); // 一次取得命令与有效标志/序号；这里没有检查命令年龄，失联许可由 Input/RobotCmd 管理。
INS_State ins{}; // 本周期 INS 快照；读取失败时变量可能保留旧值，因此使用前必须检查 ins_valid。
const bool ins_valid = MessageCenter::INS_State_Topic.ReadFresh(ins, 10000U);
```

反馈由云台在固定分频时发布，消费者可使用
`RobotCmd_GetGimbalFeedback(feedback)`：其返回值只说明 Topic 数据不超过 100 ms，
还要检查 `feedback.ins_valid` 与 `feedback.enabled`。
`enabled` 仅表示两轴快照均 ready，不等于当前控制模式允许输出，也不表示 CAN 已发送。
`Gimbal_GetStatus()` 不在反馈消息内，不能作为跨板状态接口直接使用。

## 参数配置

在 [Gimbal_Config.h](Gimbal_Config.h) 集中配置；`Gimbal_Init()` 复制默认配置，
也可在启动阶段传入一份 `Struct_Gimbal_Config`。初始化只调用一次，不等待电机、
不自动设置机械零位、不切换控制模式、不写电机持久化参数。电机端须预先设置 MIT 模式。
配置校验失败或驱动注册失败返回 false，`Gimbal_GetStatus()` 返回 CONFIG_ERROR。
电机、PID、目标和 Snapshot 由 `Gimbal.cpp` 的私有 `GimbalContext` 持有；
外部只能通过初始化、周期入口和只读状态接口访问 Application。

参数参考本地 H7_BSP 老步兵工程：Yaw 使用底盘中的云台电机参数，Pitch 使用云台参数。
只采用可直接对应的 ID、协议量程、Yaw 速度上限和 Pitch 限位；控制框架保持现有实现。

| 项目 | 默认值 | 配置要求 |
| --- | --- | --- |
| Yaw / Pitch 总线 | BoardConfig 的 gimbal_yaw_bus / gimbal_pitch_bus | 按所选板型接线 |
| 电机 ID / 反馈 Master ID | Yaw 0x03 / 0x005；Pitch 0x09 / 0x019 | 与电机端一致，反馈 ID 不得与同总线设备冲突 |
| 两轴协议量程 | ±3.14 rad、±30 rad/s、±10 N·m | 必须与电机端 PMAX/VMAX/TMAX 相同 |
| Yaw 角度 Kp / 速度上限 | 8 / 15 rad/s | 角度增益保留，速度上限参考老步兵底盘 |
| Yaw 速度 PID / 积分上限 | 全部为 0 | 保留安全默认值，尚不产生主动转矩 |
| Yaw 转矩上限 | 10 N·m | 与协议量程一致，实际输出仍受零增益约束 |
| Pitch MIT Kp / Kd / 速度上限 | 20 / 1 / 1 rad/s | 保留现有值，须实机整定 |
| Pitch 位置限位 | [-0.6981317, 0.2617994] rad（-40° 至 +15°） | 参考老步兵云台，须按实际电机零位核对 |
| Pitch 电机/姿态比例 | 1 | 保留现有正传动比，反向使用 reverse |
| IMU 角速度轴 / 符号 | Yaw Z / +1；Pitch Y / +1 | 保留现有安装约定 |

老工程的 Pitch IMU 转矩增益和 Yaw MIT 速度阻尼不等价于当前控制环增益，未套用。
这些默认值仅供配置参考，不代表当前双达妙机构已完成标定。

## 控制契约

`GimbalCmd` 的角度是 INS 姿态 rad，速度是姿态角速度前馈 rad/s。

- Yaw：最短路径角误差 → 角度比例环 → 叠加速度前馈并限幅 → 速度 PID → 转矩限幅 → `SetTorque()`。
- Pitch：`p_ref = p_motor + ratio * (pitch_ref - pitch_imu)`；
  `v_ref = v_motor + ratio * (pitch_speed_ref - gyro_pitch)`。
  使用一次快照中的电机位置/速度，按配置限幅后发出 MIT 指令，转矩前馈为零。
- 电机反馈及目标均使用经过 reverse 统一的逻辑方向，协议编码只在驱动中翻转一次。
- 姿态轴和选用的机体系角速度须与机构约定匹配；当前不是任意安装姿态的完整坐标变换器。
- LOCK 捕获并保持当前姿态，忽略随后发布的目标字段；IMU 使用新发布的目标。
- 输入边界校验外部命令与 INS 数值，DM 驱动解码合法反馈；云台控制路径只判断命令模式、INS 新鲜度及电机快照，DISABLED 始终优先停机。

## 状态与恢复

`Gimbal_GetStatus()` 返回 DISABLE、ENABLING、READY、FAULT 或 CONFIG_ERROR。
`Gimbal_Init()` 不发送使能；收到活动模式后才启动就绪流程。现有 RobotCmd 启动默认
发布 LOCK，因此打开云台编译选项后会自动进入此流程，不能把示例参数当作上板标定结果。

- INS 必须不超过 10 ms；两轴运动反馈必须小于 100 ms，在线判断不等待 StatusTask。INS 发布端拒绝非有限姿态或角速度。
- 云台每周期读取命令、INS 与两轴快照；活动模式请求两轴使能，两轴 ready 后立即捕获当前姿态并执行控制。没有就绪超时、退避或稳定窗口。
- `Gimbal_GetStatus()` 根据初始化结果、当前命令、INS 新鲜度和两轴 `ready/fault` 给出 DISABLE、ENABLING、READY、FAULT 或 CONFIG_ERROR；状态只用于观察，不驱动恢复流程。CAN 软件周期槽是否接受目标不改变云台状态。
- DMMotor 的 `RequestEnabled()` 处理首次请求和状态边沿：`false→true` 立即尝试一次 Enable，不主动发布安全目标；首次 `false` 或 `true→false` 立即尝试发布安全目标并提交一次 Disable。相同状态重复请求不执行收发；存在待提交项时返回 `false`。云台只有在两轴 ready 后才写正常目标，DMMotor 的 `SetXXX()` 在未 ready 时仍自动安全化。
- 100 Hz StatusTask 调用 `ServiceAll()`：补交失败的安全目标；在线且无故障时补交失败的当前协议命令，并在反馈与请求不一致时再次提交。离线或故障时不新增 Enable/Disable。详细提交语义见 [DM 电机驱动](../../Device/Peripheral/Motor/DMmotor/dmmotor.md)。已进入硬件 FIFO 的帧由 FDCAN Auto Retransmission 处理总线级重发。
- DISABLED、故障或初始化部分失败时，对已注册电机调用 `RequestEnabled(false)`；
  DMMotor 在首次请求或 `true→false` 边沿立即尝试覆盖周期槽为零刚度/阻尼/转矩并提交一次失能，相同请求不重复发布；失败项交给低频服务补交，在线反馈仍显示使能时继续纠正失能。离线时不反复刷失能命令；停止帧不能
  保证在物理断线时送达，也不会清除已经进入硬件 FIFO 的帧。
- 活动模式下按当前设备状态恢复。恢复先清空 PID 历史并捕获当前姿态；IMU 等待 READY 后重新
  发布目标，LOCK 直接保持新捕获的姿态，故障前目标不会重放。
- Daemon 只判断反馈活性；DMMotor 根据云台请求维护协议状态，不自动 ClearError。

`GimbalFeedback` 仍为 100 Hz，字段布局不变。`enabled` 表示两轴电机均 ready；
`ins_valid=false` 时发布零姿态/速度。使能命令提交成功不代表已使能。

云台本身不做命令来源仲裁或命令心跳检查；输入许可与来源选择由 Input/RobotCmd 处理。
无新命令时保持最后模式和目标。自动恢复后需要新目标，不能替代上层遥控失联策略。

## 如果要重新开发这个 Application

### 只移植现有实现

1. 选择 SingleBoard 或 GimbalBoard，核对 BoardConfig 的总线接线。SingleBoard 需要
   `H7_APP_GIMBAL=ON` 才编译硬件控制路径；GimbalBoard 固定启用。
2. 在 `Gimbal_Config.h` 填入电机端实际 ID、Master ID、协议量程、方向和机构限位。
   总线资源归 BoardConfig，机构参数归云台配置。角度和速度进入控制前统一为 rad、rad/s。
3. 核对 IMU 安装方向和电机反馈方向，区分 INS 姿态与编码器位置。修改减速比或安装轴时，
   一并检查 `Gyro()`、Pitch 目标换算和机械限位对应的坐标。
4. 在已有任务初始化位置使用默认配置，或替换为下面的自定义配置调用。
   保留原来的周期调用位置和 RobotCmd 命令路径。
5. 检查 DISABLED、INS 过期、电机离线、部分初始化失败和恢复后的行为，再逐步整定增益。
   当前 Yaw 速度 PID 全零，构建成功或状态 READY 都不意味着 Yaw 已能闭环。

自定义配置示例：在所选 ControlTask 中**替换**原 `Gimbal_Init()` 调用，不要追加第二次调用。
局部配置会被复制，退出初始化代码后不需要继续保存该对象。

```cpp
#if GIMBAL
Struct_Gimbal_Config config = Gimbal_Default_Config();
config.yaw.reverse = true;
const bool gimbal_initialized = Gimbal_Init(config);
#endif
```

### 替换内部控制算法

保留公开入口和消息契约，就可以在 `Gimbal.cpp` 内替换控制器，而不要求其他模块
访问云台私有对象。下表列出当前函数的职责；这些内部函数不是框架必须规定的函数名，
新实现可以改名或合并，但相应职责需要有人承担。

| 当前内部函数/对象 | 重新开发时如何处理 |
| --- | --- |
| `GimbalContext` | 放置本模块拥有的电机、控制器、输入快照、目标和恢复状态；保持私有静态生命周期 |
| `ConfigValid()` | 校验新算法需要的参数与资源约束；增加参数时同步配置和校验 |
| `ResetControllers()` | 重置积分、微分及目标历史；同时在初始化与恢复路径核对调用 |
| `PrepareControl()` | 集中处理运行许可、使能等待与恢复捕获；更新入口每周期调用一次，不发布反馈 |
| `CapturePose()` | 捕获安全保持目标、清空前馈并记住命令序号，避免恢复后追赶旧目标 |
| `Control()` | 计算双轴控制并通过 Device 提交限幅目标，调用前须完成运行许可与目标处理 |
| `UpdateTarget()` | 实现模式切换和目标接收；如改变 LOCK/IMU 语义，同步所有命令产生者 |
| `Gyro()` | 按安装约定取角速度；复杂安装姿态需要在这里或明确的坐标转换层处理 |
| `Stop()` | 对已注册设备请求安全停机，覆盖初始化失败、禁用及故障路径 |
| `PublishFeedback()` | 保持 100 Hz 反馈及字段含义，失效 INS 不冒充有效姿态 |
| `Gimbal_Update()` | 读取快照后调用一次 PrepareControl，获准才更新目标与输出，最后统一发布反馈 |

更换 Yaw PID、加入前馈或轨迹规划通常只涉及 `Control()`、控制器状态和参数。
改变两轴耦合或坐标系时，还需要核对目标转换、捕获姿态与反馈定义。
控制周期内不得等待反馈、延时、动态分配或输出阻塞日志；周期变化时也要同步控制器时间参数
与反馈分频，不能仅改变任务唤醒频率。

### 内部实现代码例程

点击各项标题展开代码；示例只保留控制意图、单位和接口约束的必要注释。

以下十一段取自当前 [Gimbal.cpp](Gimbal.cpp)，与上表逐项对应。它们使用同一个私有
`ctx`，需要文件已有的头文件、常量、`Clamp()` 和公开初始化代码；不是十个独立程序。
在原文件中替换对应实现即可，不要在另一个文件重复创建电机对象或重复定义函数。
除反馈与更新入口外，电机相关实现处于 `#if GIMBAL` 内；Context 同样保留条件编译。

<details>
<summary>GimbalContext：私有状态与设备所有权</summary>

下面保留当前完整状态布局。设备对象具有静态生命周期，外部通过公开入口和消息访问；换算法时只在此增加所需控制器状态。

```cpp
namespace
{
struct GimbalContext
{
    INS_State ins{}; // 本周期 INS 快照；读取失败时变量可能保留旧值，因此使用前必须检查 ins_valid。
    bool ins_valid = false; // 表达传感器数据的新鲜度，不等于电机就绪或整车允许运动。
    uint8_t feedback_divider = 0U; // 按更新次数分频；1 kHz 下十次为 10 ms，任务周期改变时必须同步分频。
#if GIMBAL
    Struct_Gimbal_Config config{}; // 持有参数副本；总线归 BoardConfig，机构增益与限幅归 Application。
    Class_DMMotor yaw_motor; // 电机必须保持静态生命周期，注册的 CAN 回调和驱动维护列表会持续访问它。
    Class_DMMotor pitch_motor;
    Class_PID yaw_angle_pid;
    Class_PID yaw_speed_pid;
    Struct_DMMotor_Snapshot yaw_snapshot{};
    Struct_DMMotor_Snapshot pitch_snapshot{};
    GimbalCmd command{};
    GimbalMode last_mode = GimbalMode::DISABLED;
    bool initialized = false;
    bool was_ready = false; // 用于恢复捕获；没有这个历史状态，重连后可能直接追赶故障前目标。
    float target_yaw_angle_rad = 0.0f; // 保存 Yaw 姿态目标，单位 rad。
    float target_pitch_angle_rad = 0.0f; // 保存 Pitch 姿态目标，单位 rad。
    float target_yaw_speed_rad_s = 0.0f; // 保存 Yaw 角速度前馈，单位 rad/s。
    float target_pitch_speed_rad_s = 0.0f; // 保存 Pitch 角速度前馈，单位 rad/s。
    bool yaw_registered = false;
    bool pitch_registered = false;
    uint32_t target_sequence = 0U; // 区分新发布与旧目标；数值相同也可以是新命令，判断依据是 Topic 序号。
#endif
};

GimbalContext ctx;
}
```

</details>

<details>
<summary>ConfigValid()：配置校验</summary>

应用层检查增益、限幅、轴向与 ID 冲突。新增参数时在这里添加对应条件；电机协议量程和总线合法性仍由 Device 初始化检查。

```cpp
bool ConfigValid(const Struct_Gimbal_Config& c)
{
    const float nonnegative[] = {c.yaw_angle_kp, c.yaw_speed_kp, c.yaw_speed_ki,
                                 c.yaw_speed_kd, c.yaw_integral_limit, c.pitch_kp, c.pitch_kd};
    for (float value : nonnegative)
    {
        if (!std::isfinite(value) || value < 0)
        {
            return false;
        }
    }
    const float positive[] = {c.yaw_speed_limit, c.yaw_torque_limit,
                              c.pitch_speed_limit, c.pitch_motor_per_imu};
    for (float value : positive)
    {
        if (!std::isfinite(value) || value <= 0)
        {
            return false;
        }
    }
    return !(c.yaw.bus == c.pitch.bus &&
             (c.yaw.id == c.pitch.id || c.yaw.feedback_id == c.pitch.feedback_id)) &&
           c.yaw_gyro_axis <= GimbalGyroAxis::Z && c.pitch_gyro_axis <= GimbalGyroAxis::Z &&
           (c.yaw_gyro_sign == 1 || c.yaw_gyro_sign == -1) &&
           (c.pitch_gyro_sign == 1 || c.pitch_gyro_sign == -1) &&
           c.yaw_integral_limit <= c.yaw_torque_limit && // 积分输出限幅不能超过总转矩上限。
           std::isfinite(c.pitch_min) && std::isfinite(c.pitch_max) &&
           c.pitch_min < c.pitch_max;
}
```

</details>

<details>
<summary>ResetControllers()：清空历史并设置增益</summary>

直接赋予新的 PID 值对象清空历史，再调用 Init 设置增益。更换控制器时，同步替换其重置逻辑，避免恢复后沿用故障前的积分。

```cpp
void ResetControllers()
{
    // PID::Init 保留历史，因此先重建值对象，清除积分、微分及目标历史。
    ctx.yaw_angle_pid = Class_PID{}; // 重建对象是为了清除历史；只调用 Init 不足以完成控制器复位。
    ctx.yaw_speed_pid = Class_PID{}; // 同时清除速度环历史，避免恢复时旧积分直接产生转矩。
    ctx.yaw_angle_pid.Init(ctx.config.yaw_angle_kp, 0, 0, 0, 0, ctx.config.yaw_speed_limit); // 设置纯比例角度环，输出限幅为角速度上限。
    ctx.yaw_speed_pid.Init(ctx.config.yaw_speed_kp, ctx.config.yaw_speed_ki,
                           ctx.config.yaw_speed_kd, 0, ctx.config.yaw_integral_limit, ctx.config.yaw_torque_limit); // 设置微分增益、零死区及积分/转矩限幅。
}
```

</details>

<details>
<summary>CapturePose()：建立保持目标</summary>

由首次就绪、恢复或进入 LOCK 的路径调用。sequence 来自当前命令快照；保存此序号后，相同旧序号不会再次成为 IMU 目标。调用前 INS 必须有效。

```cpp
void CapturePose(uint32_t sequence)
{
    ResetControllers(); // 清空控制器历史，避免旧积分影响保持姿态。
    ctx.target_yaw_angle_rad = ctx.ins.yaw_rad;
    ctx.target_pitch_angle_rad = ctx.ins.pitch_rad;
    ctx.target_yaw_speed_rad_s = ctx.target_pitch_speed_rad_s = 0;
    // 恢复前已发布的目标全部丢弃；IMU 只接受之后的新序号。
    ctx.target_sequence = sequence; // 把恢复时的命令标记为已处理，禁止自动重放其旧目标。
}
```

</details>

<details>
<summary>Control()：双轴控制计算与输出</summary>

PrepareControl 通过并更新目标后调用。Yaw 输出转矩，Pitch 输出 MIT 位置/速度目标，
使用本周期快照并保留既有方向、限幅与单位约定。

```cpp
void Control(const Struct_DMMotor_Snapshot& pitch)
{
    // Yaw 复用现有 PID：最短角误差生成角速度，再由速度环生成转矩。
    const float error = std::remainder(ctx.target_yaw_angle_rad - ctx.ins.yaw_rad, 2 * GIMBAL_PI);
    ctx.yaw_angle_pid.Set_Target(error);
    ctx.yaw_angle_pid.Set_Now(0);
    ctx.yaw_angle_pid.TIM_Calculate_PeriodElapsedCallback();
    const float speed = ctx.yaw_angle_pid.Get_Out() + ctx.target_yaw_speed_rad_s;
    ctx.yaw_speed_pid.Set_Target(Clamp(speed, -ctx.config.yaw_speed_limit, ctx.config.yaw_speed_limit));
    ctx.yaw_speed_pid.Set_Now(Gyro(ctx.config.yaw_gyro_axis, ctx.config.yaw_gyro_sign));
    ctx.yaw_speed_pid.TIM_Calculate_PeriodElapsedCallback();
    const float torque = ctx.yaw_speed_pid.Get_Out();
    // Pitch 将 INS 姿态/角速度误差换算为电机目标；位置、速度均基于同一次反馈快照。
    const float position = pitch.feedback.position + ctx.config.pitch_motor_per_imu *
                                                         (ctx.target_pitch_angle_rad - ctx.ins.pitch_rad);
    const float velocity = pitch.feedback.velocity + ctx.config.pitch_motor_per_imu *
                                                         (ctx.target_pitch_speed_rad_s - Gyro(ctx.config.pitch_gyro_axis, ctx.config.pitch_gyro_sign));
    (void) ctx.yaw_motor.SetTorque(Clamp(torque, -ctx.config.yaw_torque_limit, ctx.config.yaw_torque_limit));
    (void) ctx.pitch_motor.SetMIT(Clamp(position, ctx.config.pitch_min, ctx.config.pitch_max),
                                  Clamp(velocity, -ctx.config.pitch_speed_limit, ctx.config.pitch_speed_limit), ctx.config.pitch_kp, ctx.config.pitch_kd, 0);
}
```

</details>

<details>
<summary>UpdateTarget()：消费模式和新目标</summary>

ctx.command 已由更新入口从同一 message 复制。此函数处理 LOCK 进入边沿和 IMU 新序号，不自行读取或发布命令；改变目标语义时同时同步 RobotCmd。

```cpp
void UpdateTarget(const TopicSnapshot<GimbalCmd>& message)
{
    // LOCK 只在进入时捕获姿态；IMU 只接受新序号，避免恢复后重放旧目标。
    if (ctx.command.mode == GimbalMode::LOCK && ctx.last_mode != GimbalMode::LOCK)
    {
        CapturePose(message.sequence);
    }
    if (ctx.command.mode == GimbalMode::IMU && message.sequence != ctx.target_sequence)
    {
        ctx.target_yaw_angle_rad = ctx.command.yaw_angle_rad;
        ctx.target_pitch_angle_rad = ctx.command.pitch_angle_rad;
        ctx.target_yaw_speed_rad_s = ctx.command.yaw_speed_rad_s;
        ctx.target_pitch_speed_rad_s = ctx.command.pitch_speed_rad_s;
        ctx.target_sequence = message.sequence; // 消费新目标序号；恢复捕获已消费旧序号，因此恢复后需要重新发布。
    }
    ctx.last_mode = ctx.command.mode;
}
```

</details>

<details>
<summary>Gyro()：选择角速度轴</summary>

按配置选择 INS 机体系角速度并统一符号。axis/sign 已经校验，输出单位为 rad/s；复杂安装姿态应改为明确的坐标变换，不能仅靠换轴处理。

```cpp
float Gyro(GimbalGyroAxis axis, float sign)
{
    const float rates[] = {ctx.ins.gyro_x_rad_s, ctx.ins.gyro_y_rad_s,
                           ctx.ins.gyro_z_rad_s};
    return sign * rates[static_cast<unsigned>(axis)];
}
```

</details>

<details>
<summary>PrepareControl()：运行许可、使能等待与恢复</summary>

调用前已读取本周期 INS、命令与两轴快照。此函数集中处理停机、未就绪等待和恢复捕获，
返回 true 后才更新目标并计算输出；反馈由更新入口统一发布一次。

```cpp
bool PrepareControl(const TopicSnapshot<GimbalCmd>& message)
{
    if (!ctx.initialized || !message.valid ||
        ctx.command.mode == GimbalMode::DISABLED || !ctx.ins_valid ||
        ctx.yaw_snapshot.fault || ctx.pitch_snapshot.fault)
    {
        Stop();
        ctx.was_ready = false;
        ctx.last_mode = GimbalMode::DISABLED;
        ctx.yaw_snapshot = ctx.yaw_motor.GetFeedbackSnapshot();
        ctx.pitch_snapshot = ctx.pitch_motor.GetFeedbackSnapshot();
        return false;
    }

    // 每周期表达输出许可；驱动只处理请求边沿，重复使能不会覆盖正常周期目标。
    (void) ctx.yaw_motor.RequestEnabled(true);
    (void) ctx.pitch_motor.RequestEnabled(true);
    ctx.yaw_snapshot = ctx.yaw_motor.GetFeedbackSnapshot();
    ctx.pitch_snapshot = ctx.pitch_motor.GetFeedbackSnapshot();
    if (!ctx.yaw_snapshot.ready || !ctx.pitch_snapshot.ready)
    {
        // 等待两轴反馈就绪；安全输出和低频协议纠正由 DMMotor 维护。
        ctx.was_ready = false;
        return false;
    }

    if (!ctx.was_ready)
    {
        CapturePose(message.sequence);
        ctx.last_mode = ctx.command.mode;
        ctx.was_ready = true;
    }
    return true;
}
```

</details>

<details>
<summary>Stop()：覆盖部分初始化与停机</summary>

分别判断两轴注册结果，不假设两轴同时初始化成功。RequestEnabled(false) 只是表达失能请求，不代表电机已经物理停止；低频补交仍由驱动负责。

```cpp
void Stop()
{
    if (ctx.yaw_registered)
    {
        (void) ctx.yaw_motor.RequestEnabled(false); // 表达 Yaw 失能请求，提交不等于停止确认。
    }
    if (ctx.pitch_registered)
    {
        (void) ctx.pitch_motor.RequestEnabled(false); // 表达 Pitch 失能请求，后续补交由驱动处理。
    }
}
```

</details>

<details>
<summary>PublishFeedback()：分频反馈</summary>

每十次调用发布一次，所有更新分支都必须保持这个调用频率。INS 无效时使用零值并标记无效；消费者不得只根据零姿态判断是否有效。

```cpp
static void PublishFeedback(void)
{
    if (++ctx.feedback_divider >= 10U)
    {
        ctx.feedback_divider = 0;
        GimbalFeedback feedback{}; // 未填字段保持零，包括本单轴例程不控制的 Pitch；无效 INS 不能冒充零姿态。
        if (ctx.ins_valid)
        {
            feedback.yaw_rad = ctx.ins.yaw_rad; // 反馈 INS Yaw 姿态，不是电机编码器角度。
            feedback.pitch_rad = ctx.ins.pitch_rad; // 反馈 INS Pitch 姿态，单位 rad。
#if GIMBAL
            feedback.yaw_speed_rad_s = Gyro(ctx.config.yaw_gyro_axis, ctx.config.yaw_gyro_sign);
            feedback.pitch_speed_rad_s = Gyro(ctx.config.pitch_gyro_axis, ctx.config.pitch_gyro_sign);
#else
            feedback.yaw_speed_rad_s = ctx.ins.gyro_z_rad_s;
            feedback.pitch_speed_rad_s = ctx.ins.gyro_y_rad_s;
#endif
        }
        feedback.ins_valid = ctx.ins_valid;
#if GIMBAL
        feedback.enabled = ctx.yaw_snapshot.ready && ctx.pitch_snapshot.ready; // 仅表示两轴都 ready，不表示控制输出被许可或 CAN 已发送。
#endif
        MessageCenter::Gimbal_Feedback_Topic.Publish(feedback); // Latest-Value 通道只保留最新反馈，不能用于需要逐条保留的事件。
    }
}
```

</details>

<details>
<summary>Gimbal_Update()：串联完整周期</summary>

下面给出当前完整入口，展示停机、等待 ready、恢复捕获和正常控制的顺序。安全与恢复由 PrepareControl 集中处理；入口最后统一调用 PublishFeedback，不在等待 ready 时阻塞。

```cpp
void Gimbal_Update(void)
{
    ctx.ins_valid = MessageCenter::INS_State_Topic.ReadFresh(ctx.ins, GIMBAL_INS_MAX_AGE_US);
#if GIMBAL
    const auto message = MessageCenter::Gimbal_Command_Topic.ReadWithMeta();
    ctx.command = message.valid ? message.data : GimbalCmd{};
    ctx.yaw_snapshot = ctx.yaw_motor.GetFeedbackSnapshot();
    ctx.pitch_snapshot = ctx.pitch_motor.GetFeedbackSnapshot();

    if (PrepareControl(message))
    {
        UpdateTarget(message);

        Control(ctx.pitch_snapshot);
    }
#endif
    PublishFeedback();
}
```

</details>

### 完整最简例程：单轴 INS 位置环 → 达妙速度模式

下面给出完整的单轴 Application 例程，按函数分段展示，每段可以单独展开。
依次排列这些代码段即可组成完整的 `Gimbal.cpp`，无需修改或拼接前面的双轴例程。
它沿用当前 `Gimbal.h`、`Gimbal_Config.h` 与三条 Topic，由现有 ControlTask 调用。
本节只提供例程，仓库的双轴生产实现保持原样。
例程使用明确的变量类型和展开的 `if/else`；`::` 是作用域限定，`.` 调用对象接口，
初始化参数中的 `&` 表示引用，这些写法来自项目现有接口。

控制链为 `INS Yaw 角误差 → 单位置 PID → SetSpeed(rad/s)`，应用层不计算速度环。
电机端须提前配置为速度模式；驱动 Init 不自动修改电机端模式。
假设 Yaw 对应 INS Z 轴、直接传动，`reverse` 与实际运动方向匹配。
默认增益与速度上限来自配置，可按机构调整 `yaw_angle_kp` 和 `yaw_speed_limit`，
首次验证可将速度上限设为 1 rad/s。Ki、Kd 为零，先学习比例位置环。

这个示例只注册 Yaw，完全不使用 Pitch 配置；`GimbalFeedback.enabled` 表示单轴 ready，
Pitch 反馈字段保持零。因此用于实际单轴工程时，反馈消费者也必须采用单轴含义。
LOCK 捕获姿态，IMU 在首次就绪或恢复后需新发布序号的目标；速度前馈字段不参与运算。
现有 100 Hz StatusTask 的 `Class_DMMotor::ServiceAll()` 仍需运行。

<details>
<summary>1. 头文件与私有状态</summary>

所有函数共享一个静态电机实例与状态，设备生命周期覆盖注册回调的使用期。

```cpp
#include "Gimbal.h"
#include "message_center.h"
#include <cmath>
#if GIMBAL
#include "alg_pid.h"
#include "dmmotor.h"
#endif
namespace
{
const float kTwoPi = 6.28318530718f; // 一个完整旋转，单位 rad。
INS_State ins; // 本周期 INS 快照；读取失败时变量可能保留旧值，因此使用前必须检查 ins_valid。
bool ins_valid = false; // 表达传感器数据的新鲜度，不等于电机就绪或整车允许运动。
uint8_t feedback_divider = 0U; // 按更新次数分频；1 kHz 下十次为 10 ms，任务周期改变时必须同步分频。
#if GIMBAL
Struct_Gimbal_Config config; // 持有参数副本；总线归 BoardConfig，机构增益与限幅归 Application。
Class_DMMotor yaw_motor; // 电机必须保持静态生命周期，注册的 CAN 回调和驱动维护列表会持续访问它。
Class_PID position_pid; // 输入角误差 rad，输出电机速度 rad/s；电机内部承担速度闭环。
Struct_DMMotor_Snapshot snapshot; // 同一次读取包含运动反馈与 ready/fault，避免跨时刻拼接状态。
GimbalCmd command;
bool registered = false; // 与 initialized 分开记录：部分初始化失败时，仍需对已注册设备执行停机。
bool initialized = false;
bool was_ready = false; // 用于恢复捕获；没有这个历史状态，重连后可能直接追赶故障前目标。
GimbalMode last_mode = GimbalMode::DISABLED;
uint32_t target_sequence = 0U; // 区分新发布与旧目标；数值相同也可以是新命令，判断依据是 Topic 序号。
float target_rad = 0.0f; // INS 姿态坐标下的保持/跟踪目标，不是电机编码器位置。
```

</details>

<details>
<summary>2. Capture()：捕获姿态和重置位置环</summary>

首次就绪、恢复或进入 LOCK 时捕获安全保持目标，并清空 PID 历史。

```cpp
void Capture(uint32_t sequence)
{
    Class_PID fresh_pid;
    position_pid = fresh_pid; // PID::Init 不清除全部历史，先重建对象；以后加入 Ki/Kd 也不会沿用故障前历史。
    position_pid.Init(config.yaw_angle_kp, 0.0f, 0.0f, 0.0f, 0.0f, config.yaw_speed_limit, 0.001f); // 参数依次为 Kp/Ki/Kd/Kf、积分输出限幅、总输出限幅、dt；本例 Kp 单位 1/s，限幅 rad/s，dt=1 ms。
    target_rad = ins.yaw_rad; // 恢复时用当前姿态保持，避免突然转向恢复前的旧目标。
    target_sequence = sequence; // 捕获时消费当前序号，IMU 必须收到之后的新发布才更新目标。
}
```

</details>

<details>
<summary>3. PrepareControl()：安全检查、掉线等待与恢复</summary>

更新入口每周期只调用一次。集中检查初始化、命令模式/数值、INS 时效和设备故障，再处理停机、使能、未就绪等待与恢复捕获。驱动的 ready 包含在线判定，掉线会进入等待路径；不在这里循环等待反馈，也不增加另一套 Daemon。返回 true 才允许执行目标与控制计算。

```cpp
bool PrepareControl(const TopicSnapshot<GimbalCmd>& message)
{
    bool safe = true; // 只使用更新入口已经读取的本周期数据，不另建掉线计时器。

    if (!initialized) // 应用尚未完成有效配置和设备注册，不能正常输出。
    {
        safe = false;
    }
    if (!message.valid) // 命令 Topic 尚无有效发布，不能沿用未建立的目标。
    {
        safe = false;
    }
    bool active = false;
    if (command.mode == GimbalMode::LOCK) // LOCK 是保持姿态模式，运行许可中将它视为活动模式。
    {
        active = true;
    }
    else if (command.mode == GimbalMode::IMU) // IMU 模式使用外部姿态目标，需要校验并接收新目标。
    {
        active = true;
    }
    if (!active) // 只有 LOCK/IMU 允许活动控制，禁用或未知模式不给运行许可。
    {
        safe = false;
    }
    if (command.mode == GimbalMode::IMU) // IMU 模式使用外部姿态目标，需要校验并接收新目标。
    {
        if (!std::isfinite(command.yaw_angle_rad)) // 拒绝 NaN 或无穷大的目标，避免异常数值进入 PID。
        {
            safe = false;
        }
    }
    if (!ins_valid) // INS 未发布或已经超过时效，不能使用旧姿态继续闭环。
    {
        safe = false;
    }
    if (!std::isfinite(ins.yaw_rad)) // 即使时间戳新鲜，姿态数值异常也必须禁止正常控制。
    {
        safe = false;
    }
    if (snapshot.fault) // 达妙反馈报告协议故障，进入安全处理而非正常输出。
    {
        safe = false;
    }

    if (!safe) // 运行许可检查有失败项，统一执行停机并清除恢复历史。
    {
        if (registered) // 只对注册成功的电机处理停机，兼容部分初始化失败。
        {
            yaw_motor.RequestEnabled(false); // 表达失能请求，协议补交仍由驱动维护。
        }
        was_ready = false; // 使恢复时重新捕获姿态，避免追赶旧目标。
        last_mode = GimbalMode::DISABLED;
        snapshot = yaw_motor.GetFeedbackSnapshot();
        return false;
    }
    yaw_motor.RequestEnabled(true);
    snapshot = yaw_motor.GetFeedbackSnapshot(); // online/ready 由驱动按反馈时效和协议状态推导。
    if (!snapshot.ready) // 设备掉线或尚未满足使能条件，跳过控制，后续周期继续等待。
    {
        was_ready = false; // 掉线或未使能时跳过控制，下一周期继续检查，不阻塞等待。
        return false;
    }
    if (!was_ready) // 首次就绪或掉线后恢复，重新捕获当前姿态而非追赶旧目标。
    {
        Capture(message.sequence); // 首次就绪或恢复先保持当前姿态，消费恢复前的目标序号。
        last_mode = command.mode;
        was_ready = true;
    }
    return true;
}
```

</details>

<details>
<summary>4. UpdateTarget()：模式与目标处理</summary>

运行许可通过后处理正常模式切换和新发布目标；故障后的首次捕获由 PrepareControl 完成。

```cpp
void UpdateTarget(const TopicSnapshot<GimbalCmd>& message)
{
    if (command.mode == GimbalMode::LOCK) // LOCK 是保持姿态模式，运行许可中将它视为活动模式。
    {
        if (last_mode != GimbalMode::LOCK) // 当前已是 LOCK，但上一周期不是；只在进入边沿重新捕获姿态。
        {
            Capture(message.sequence); // 正常运行中进入 LOCK 时也重新捕获姿态。
        }
    }
    if (command.mode == GimbalMode::IMU) // IMU 模式使用外部姿态目标，需要校验并接收新目标。
    {
        if (message.sequence != target_sequence) // 只有新发布序号才能更新 IMU 目标，恢复前旧目标不重放。
        {
            target_rad = command.yaw_angle_rad; // INS 姿态目标，不是电机编码器位置。
            target_sequence = message.sequence; // 同值目标必须重新发布才会产生新序号。
        }
    }
    last_mode = command.mode;
}
```

</details>

<details>
<summary>5. Control()：INS 位置环与电机输出</summary>

仅在 PrepareControl 通过且目标更新后调用，计算本周期的位置环并提交电机目标。
末尾结束第一段开始的硬件条件编译。

```cpp
void Control()
{
    const float error_rad = std::remainder(target_rad - ins.yaw_rad, kTwoPi); // 角误差按 2π 折回最短路径；例如跨过 ±π 时不会绕远路，不适用于指定多圈旋转。
    position_pid.Set_Target(error_rad); // 先计算绕回后的误差再送入 PID，避免把多圈原始角度直接相减产生跳变。
    position_pid.Set_Now(0.0f); // PID 内部计算 Target-Now；这里 Target 已是误差，所以 Now 必须为零，不能再减一次 INS。
    position_pid.TIM_Calculate_PeriodElapsedCallback(); // 调用频率必须与 Init 的 dt 一致；漏周期或改调度频率时要重新核对时间参数。
    float speed_rad_s = position_pid.Get_Out();
    yaw_motor.SetSpeed(speed_rad_s); // Get_Out 已按 yaw_speed_limit 限幅，输出 rad/s；返回值只表示软件提交，当前例程沿用驱动低频补交。
}
#endif
```

</details>

<details>
<summary>6. PublishFeedback()：100 Hz 反馈</summary>

更新入口始终调用一次，不由安全函数单独发布。该段最后关闭私有命名空间。

```cpp
void PublishFeedback()
{
    feedback_divider = feedback_divider + 1U;
    if (feedback_divider < 10U) // 1 kHz 下尚未达到 10 ms 发布间隔，保持反馈为 100 Hz。
    {
        return;
    }
    feedback_divider = 0U;
    GimbalFeedback feedback; // 未填字段保持零，包括本单轴例程不控制的 Pitch；无效 INS 不能冒充零姿态。
    feedback.ins_valid = ins_valid; // 消费者必须同时检查此标志；Topic 新鲜不代表内部 INS 数据有效。
    if (ins_valid) // 只将有效 INS 姿态填入反馈，失效时保持零值并显式标记无效。
    {
        feedback.yaw_rad = ins.yaw_rad; // 单轴反馈 INS Yaw，单位 rad。
#if GIMBAL
        feedback.yaw_speed_rad_s = config.yaw_gyro_sign * ins.gyro_z_rad_s; // 本例固定 Z 轴，速度单位 rad/s。
#else
        feedback.yaw_speed_rad_s = ins.gyro_z_rad_s;
#endif
    }
#if GIMBAL
    feedback.enabled = snapshot.ready; // 本例表示唯一 Yaw 电机 ready，已改变原两轴语义。
#endif
    MessageCenter::Gimbal_Feedback_Topic.Publish(feedback); // Latest-Value 通道只保留最新反馈，不能用于需要逐条保留的事件。
}
} // namespace
```

</details>

<details>
<summary>7. Gimbal_Init()：单轴速度模式初始化</summary>

仅启动时调用一次，注册 Yaw 并返回结果；电机端仍需提前设为速度模式。

增益必须有限且非负，速度上限必须有限且为正，否则 NaN 或异常限幅可能进入输出。
本例固定 Z 轴，其他安装姿态会被拒绝，不能只修改配置就假定坐标变换已经完成。
驱动注册成功并不表示反馈在线或电机使能；正常输出还需等待更新入口确认 ready。

```cpp
#if GIMBAL
bool Gimbal_Init(const Struct_Gimbal_Config& requested)
{
    config = requested; // 复制参数，调用者的局部配置可以在初始化后销毁；总线指向的 HAL 对象仍须长期有效。
    initialized = false;
    if (!std::isfinite(config.yaw_angle_kp)) // 位置增益必须是有限数，拒绝 NaN 和正负无穷。
    {
        return false;
    }
    if (config.yaw_angle_kp < 0.0f) // 本例不接受负位置增益，方向应通过电机与坐标约定处理。
    {
        return false;
    }
    if (!std::isfinite(config.yaw_speed_limit)) // 速度上限不能是 NaN 或无穷，否则限幅失去意义。
    {
        return false;
    }
    if (config.yaw_speed_limit <= 0.0f) // 速度上限必须大于零，才形成有效的对称输出限幅。
    {
        return false;
    }
    if (config.yaw_gyro_axis != GimbalGyroAxis::Z) // 本例仅实现 Z 轴 Yaw，不支持仅靠改枚举适配其他安装姿态。
    {
        return false;
    }
    bool valid_sign = false;
    if (config.yaw_gyro_sign == 1.0f) // 接受与约定一致的角速度反馈正方向。
    {
        valid_sign = true;
    }
    else if (config.yaw_gyro_sign == -1.0f) // 接受反向角速度反馈标定，只允许正一或负一。
    {
        valid_sign = true;
    }
    if (!valid_sign) // 方向符号不是正一或负一，配置无效。
    {
        return false;
    }
    registered = yaw_motor.Init(config.yaw.bus, config.yaw.id, config.yaw.feedback_id, // id 用于电机命令，feedback_id 用于接收回调；同总线反馈 ID 不得与其他设备冲突。
        Enum_DMMotor_Mode::SPEED, config.yaw.reverse, config.yaw.position_max, // SPEED 决定驱动协议与安全输出；reverse 统一逻辑方向，Init 不修改电机端模式。
        config.yaw.velocity_max, config.yaw.torque_max); // 协议量程必须匹配电机参数，不能把协议速度量程当作应用控制限速。
    initialized = registered;
    return initialized;
}
```

</details>

<details>
<summary>8. Gimbal_GetStatus()：读取单轴状态</summary>

从最近更新的快照推导状态，不执行控制。末尾结束公开硬件接口的条件编译。

状态优先级是初始化错误、禁用、输入/设备故障、就绪判断。读取不会刷新 Topic 或电机，
也不能替代周期入口；跨任务观察优先读取反馈 Topic。READY 只表明设备就绪，
不说明 PID 已整定、目标已发送到硬件或机械方向正确。

```cpp
Enum_Gimbal_Status Gimbal_GetStatus(void)
{
    if (!initialized) // 应用尚未完成有效配置和设备注册，不能正常输出。
    {
        return Gimbal_Status_CONFIG_ERROR;
    }
    if (command.mode == GimbalMode::DISABLED) // 禁用命令优先于在线与就绪判断，状态报告为禁用。
    {
        return Gimbal_Status_DISABLE;
    }
    if (!ins_valid) // INS 未发布或已经超过时效，不能使用旧姿态继续闭环。
    {
        return Gimbal_Status_FAULT;
    }
    if (snapshot.fault) // 达妙反馈报告协议故障，进入安全处理而非正常输出。
    {
        return Gimbal_Status_FAULT;
    }
    if (snapshot.ready) // 就绪只代表反馈与本地/协议许可满足，不保证参数已整定或目标已执行。
    {
        return Gimbal_Status_READY;
    }
    else
    {
        return Gimbal_Status_ENABLING;
    }
}
#endif
```

</details>

<details>
<summary>9. Gimbal_Update()：主逻辑入口</summary>

先读取命令、INS 和设备快照，再调用一次 PrepareControl；获得许可后更新目标并调用 Control 计算位置 PID 与速度输出。所有路径最后统一发布反馈，便于直接看清主流程。

```cpp
void Gimbal_Update(void)
{
    ins_valid = MessageCenter::INS_State_Topic.ReadFresh(ins, 10000U); // 10000 的单位是 us；失败后即使 ins 仍有旧值，也不得继续正常控制。
#if GIMBAL
    TopicSnapshot<GimbalCmd> message = MessageCenter::Gimbal_Command_Topic.ReadWithMeta(); // 一次取得命令与有效标志/序号；这里没有检查命令年龄，失联许可由 Input/RobotCmd 管理。
    if (message.valid) // 使用已发布命令；否则采用默认 DISABLED，避免沿用旧活动命令。
    {
        command = message.data;
    }
    else
    {
        GimbalCmd default_command; // 成员默认值使模式为 DISABLED，不沿用之前的活动命令。
        command = default_command;
    }
    snapshot = yaw_motor.GetFeedbackSnapshot(); // 读取驱动对新鲜度、协议状态与请求的判断；软件提交成功不能代替 ready。
    if (PrepareControl(message)) // 安全与恢复处理允许控制后，才更新目标并计算电机输出。
    {
        UpdateTarget(message);

        Control();
    }
#endif
    PublishFeedback(); // 每周期只调用一次，故障、等待和正常路径共享同一反馈分频。
}
```

</details>

### 更换电机或改变框架接口

更换电机时在云台 `Init()` 中显式注册新的 Device，在更新中读取其快照并提交输出。
同步替换停机、就绪/故障判断和低频维护依赖；若新驱动不使用达妙协议，不要沿用达妙
`ready` 或 `ServiceAll()` 的假设。驱动负责协议编码、CAN 接收和设备状态，Application
负责控制算法及运行许可，不在云台中直接操作 HAL Handle 或解析 CAN 字节。

新增源文件必须在根 CMake 显式注册，检查 SingleBoard、GimbalBoard 的源码选择和
include 路径，避免把云台设备引入 ChassisBoard。原路径与入口保持不变时无需另建调度层。

如果现有 `GimbalCmd` / `GimbalFeedback` 足够表达新算法，应保留字段和单位。
确实需要新增模式或字段时，先修改消息类型，再同步 Input/RobotCmd 的目标生成、
变化判断、云台消费和反馈读取方；涉及线上消息时还需核对 Transport 的编码、解码与双方协议。
详细规则见 [Application 指南](../README.md) 和
[Message Center 指南](../../System/MessageCenter/README.md)。

### GM6020 最简位置例程：INS 位置环 + 驱动速度环

本例将上面的单轴 Yaw 达妙方案改成 GM6020，按函数提供完整代码。各段依次组合可构成
独立的 `Gimbal.cpp`；不要与前面的达妙例程同时编译，也不会修改仓库生产代码。

控制链是 `INS Yaw 角误差 → Application 位置 PID → rad/s 速度目标 → DJI 驱动速度 PID → GM6020 电压协议指令`。
GM6020 没有达妙 SPEED 模式；位置环输出的速度不能直接当作 GM6020 电压或电流指令。
因此保留一个最简单的驱动速度环，不启用额外的驱动角度环或电流环。
本例仍采用 LOCK/IMU、PrepareControl 安全与恢复流程，Pitch 不参与控制。

电机编号在初始化中显式设为 1，必须按实际拨码更改，不使用达妙 feedback_id。
编号 1 的反馈为 0x205，VOLTAGE 控制发送组为 0x1FF；需匹配电机端协议选择。
核对同总线 DJI 电机编号及发送组占用，尤其不能与现有底盘/发射电机占用同一槽位。
如果其他电机共享同一物理发送组，应先注册全部成员并由同一个 Class_DJIMotor_Group 统一管理，
不能把它们分别绑定到多个逻辑组。本例适用于该物理组只使用这一台电机的接线。

位置增益仍使用 config.yaw_angle_kp，位置环速度上限使用 config.yaw_speed_limit；
示例速度环 Kp=100、输出上限=1000 仅用于展示参数写法，不是已验证整定值。
GM6020 速度反馈使用编码器输出轴 rad/s，INS 姿态与电机方向必须一致。
单轴反馈 enabled 只表示 Yaw ready；DJI ready 是本地输出许可与反馈在线，不是达妙协议的使能确认。

<details>
<summary>1. 头文件与私有状态（GM6020）</summary>

设备和电机组共用静态生命周期，状态仍通过框架消息和公开入口访问。

```cpp
#include "Gimbal.h"
#include "message_center.h"
#include <cmath>
#if GIMBAL
#include "alg_pid.h"
#include "dji_motor.h"
#endif
namespace
{
const float kTwoPi = 6.28318530718f; // 一个完整旋转，单位 rad。
INS_State ins; // 本周期 INS 快照；读取失败时变量可能保留旧值，因此使用前必须检查 ins_valid。
bool ins_valid = false; // 表达传感器数据的新鲜度，不等于电机就绪或整车允许运动。
uint8_t feedback_divider = 0U; // 按更新次数分频；1 kHz 下十次为 10 ms，任务周期改变时必须同步分频。
#if GIMBAL
Struct_Gimbal_Config config; // 持有参数副本；总线归 BoardConfig，机构增益与限幅归 Application。
Class_DJIMotor yaw_motor; // 静态生命周期覆盖 CAN 回调的使用期。
Class_DJIMotor_Group yaw_group; // 单电机也需要组接口统一计算并提交物理报文。
bool group_registered = false;
Class_PID position_pid; // 输入角误差 rad，输出电机速度 rad/s；电机内部承担速度闭环。
Struct_DJIMotor_Motion_Snapshot snapshot; // 同一次读取包含运动反馈与 ready/fault，避免跨时刻拼接状态。
GimbalCmd command;
bool registered = false; // 与 initialized 分开记录：部分初始化失败时，仍需对已注册设备执行停机。
bool initialized = false;
bool was_ready = false; // 用于恢复捕获；没有这个历史状态，重连后可能直接追赶故障前目标。
GimbalMode last_mode = GimbalMode::DISABLED;
uint32_t target_sequence = 0U; // 区分新发布与旧目标；数值相同也可以是新命令，判断依据是 Topic 序号。
float target_rad = 0.0f; // INS 姿态坐标下的保持/跟踪目标，不是电机编码器位置。
```

</details>

<details>
<summary>2. Capture()：捕获姿态和重置位置环（GM6020）</summary>

恢复时重建 App 位置 PID，并清除 DJI 速度环积分，先保持当前 INS 姿态。

```cpp
void Capture(uint32_t sequence)
{
    Class_PID fresh_pid;
    position_pid = fresh_pid; // PID::Init 不清除全部历史，先重建对象；以后加入 Ki/Kd 也不会沿用故障前历史。
    position_pid.Init(config.yaw_angle_kp, 0.0f, 0.0f, 0.0f, 0.0f, config.yaw_speed_limit, 0.001f); // 参数依次为 Kp/Ki/Kd/Kf、积分输出限幅、总输出限幅、dt；本例 Kp 单位 1/s，限幅 rad/s，dt=1 ms。
    yaw_motor.speed_pid.Set_Integral_Error(0.0f); // 恢复时不沿用驱动速度环的积分。
    target_rad = ins.yaw_rad; // 恢复时用当前姿态保持，避免突然转向恢复前的旧目标。
    target_sequence = sequence; // 捕获时消费当前序号，IMU 必须收到之后的新发布才更新目标。
}
```

</details>

<details>
<summary>3. PrepareControl()：安全检查、掉线等待与恢复（GM6020）</summary>

集中处理运行许可、使能、掉线等待与恢复。DJI 快照没有达妙 fault 字段；online/ready 依据反馈超时和本地输出许可判断。失能通过组接口提交零目标，不依赖达妙 ServiceAll。

```cpp
bool PrepareControl(const TopicSnapshot<GimbalCmd>& message)
{
    bool safe = true; // 只使用更新入口已经读取的本周期数据，不另建掉线计时器。

    if (!initialized) // 应用尚未完成有效配置和设备注册，不能正常输出。
    {
        safe = false;
    }
    if (!message.valid) // 命令 Topic 尚无有效发布，不能沿用未建立的目标。
    {
        safe = false;
    }
    bool active = false;
    if (command.mode == GimbalMode::LOCK) // LOCK 是保持姿态模式，运行许可中将它视为活动模式。
    {
        active = true;
    }
    else if (command.mode == GimbalMode::IMU) // IMU 模式使用外部姿态目标，需要校验并接收新目标。
    {
        active = true;
    }
    if (!active) // 只有 LOCK/IMU 允许活动控制，禁用或未知模式不给运行许可。
    {
        safe = false;
    }
    if (command.mode == GimbalMode::IMU) // IMU 模式使用外部姿态目标，需要校验并接收新目标。
    {
        if (!std::isfinite(command.yaw_angle_rad)) // 拒绝 NaN 或无穷大的目标，避免异常数值进入 PID。
        {
            safe = false;
        }
    }
    if (!ins_valid) // INS 未发布或已经超过时效，不能使用旧姿态继续闭环。
    {
        safe = false;
    }
    if (!std::isfinite(ins.yaw_rad)) // 即使时间戳新鲜，姿态数值异常也必须禁止正常控制。
    {
        safe = false;
    }

    if (!safe) // 运行许可检查有失败项，统一执行停机并清除恢复历史。
    {
        if (registered) // 只对注册成功的电机处理停机，兼容部分初始化失败。
        {
            if (group_registered) // 组绑定成功时通过组接口清零提交，避免各成员重复发送共享报文。
            {
                yaw_group.RequestEnabled(false); // 将共享报文中的本组目标清零并提交。
            }
            else
            {
                yaw_motor.RequestEnabled(false); // 组绑定失败时也清除已注册电机的指令。
            }
        }
        was_ready = false; // 使恢复时重新捕获姿态，避免追赶旧目标。
        last_mode = GimbalMode::DISABLED;
        snapshot = yaw_motor.GetMotionSnapshot();
        return false;
    }
    yaw_group.RequestEnabled(true);
    snapshot = yaw_motor.GetMotionSnapshot(); // online/ready 由驱动按反馈时效和协议状态推导。
    if (!snapshot.ready) // 设备掉线或尚未满足使能条件，跳过控制，后续周期继续等待。
    {
        was_ready = false; // 掉线或未使能时跳过控制，下一周期继续检查，不阻塞等待。
        return false;
    }
    if (!was_ready) // 首次就绪或掉线后恢复，重新捕获当前姿态而非追赶旧目标。
    {
        Capture(message.sequence); // 首次就绪或恢复先保持当前姿态，消费恢复前的目标序号。
        last_mode = command.mode;
        was_ready = true;
    }
    return true;
}
```

</details>

<details>
<summary>4. UpdateTarget()：模式与目标处理（GM6020）</summary>

LOCK 捕获当前姿态，IMU 只接收新序号目标；不改变现有命令消息字段。

```cpp
void UpdateTarget(const TopicSnapshot<GimbalCmd>& message)
{
    if (command.mode == GimbalMode::LOCK) // LOCK 是保持姿态模式，运行许可中将它视为活动模式。
    {
        if (last_mode != GimbalMode::LOCK) // 当前已是 LOCK，但上一周期不是；只在进入边沿重新捕获姿态。
        {
            Capture(message.sequence); // 正常运行中进入 LOCK 时也重新捕获姿态。
        }
    }
    if (command.mode == GimbalMode::IMU) // IMU 模式使用外部姿态目标，需要校验并接收新目标。
    {
        if (message.sequence != target_sequence) // 只有新发布序号才能更新 IMU 目标，恢复前旧目标不重放。
        {
            target_rad = command.yaw_angle_rad; // INS 姿态目标，不是电机编码器位置。
            target_sequence = message.sequence; // 同值目标必须重新发布才会产生新序号。
        }
    }
    last_mode = command.mode;
}
```

</details>

<details>
<summary>5. Control()：INS 位置环与电机输出（GM6020）</summary>

仅在 PrepareControl 通过且目标更新后调用，计算本周期的位置环并提交电机目标。
末尾结束第一段开始的硬件条件编译。

```cpp
void Control()
{
    const float error_rad = std::remainder(target_rad - ins.yaw_rad, kTwoPi); // 角误差按 2π 折回最短路径；例如跨过 ±π 时不会绕远路，不适用于指定多圈旋转。
    position_pid.Set_Target(error_rad); // 先计算绕回后的误差再送入 PID，避免把多圈原始角度直接相减产生跳变。
    position_pid.Set_Now(0.0f); // PID 内部计算 Target-Now；这里 Target 已是误差，所以 Now 必须为零，不能再减一次 INS。
    position_pid.TIM_Calculate_PeriodElapsedCallback(); // 调用频率必须与 Init 的 dt 一致；漏周期或改调度频率时要重新核对时间参数。
    float speed_rad_s = position_pid.Get_Out();
    yaw_group.Control(speed_rad_s); // Get_Out 已按 yaw_speed_limit 限幅，输出 rad/s；返回值只表示软件提交，本例明确提交组报文，不能省略组发送。
}
#endif
```

</details>

<details>
<summary>6. PublishFeedback()：100 Hz 反馈（GM6020）</summary>

100 Hz 发布 INS 姿态与单轴 ready，所有更新路径统一维护分频。该段最后关闭私有命名空间。

```cpp
void PublishFeedback()
{
    feedback_divider = feedback_divider + 1U;
    if (feedback_divider < 10U) // 1 kHz 下尚未达到 10 ms 发布间隔，保持反馈为 100 Hz。
    {
        return;
    }
    feedback_divider = 0U;
    GimbalFeedback feedback; // 未填字段保持零，包括本单轴例程不控制的 Pitch；无效 INS 不能冒充零姿态。
    feedback.ins_valid = ins_valid; // 消费者必须同时检查此标志；Topic 新鲜不代表内部 INS 数据有效。
    if (ins_valid) // 只将有效 INS 姿态填入反馈，失效时保持零值并显式标记无效。
    {
        feedback.yaw_rad = ins.yaw_rad; // 单轴反馈 INS Yaw，单位 rad。
#if GIMBAL
        feedback.yaw_speed_rad_s = config.yaw_gyro_sign * ins.gyro_z_rad_s; // 本例固定 Z 轴，速度单位 rad/s。
#else
        feedback.yaw_speed_rad_s = ins.gyro_z_rad_s;
#endif
    }
#if GIMBAL
    feedback.enabled = snapshot.ready; // 本例表示唯一 Yaw 电机 ready，已改变原两轴语义。
#endif
    MessageCenter::Gimbal_Feedback_Topic.Publish(feedback); // Latest-Value 通道只保留最新反馈，不能用于需要逐条保留的事件。
}
} // namespace
```

</details>

<details>
<summary>7. Gimbal_Init()：单轴速度模式初始化（GM6020）</summary>

先注册 GM6020，再注册单电机组；位置环属于 App，速度环由 DJI 配置启用。

```cpp
#if GIMBAL
bool Gimbal_Init(const Struct_Gimbal_Config& requested)
{
    config = requested; // 复制参数，调用者的局部配置可以在初始化后销毁；总线指向的 HAL 对象仍须长期有效。
    initialized = false;
    if (!std::isfinite(config.yaw_angle_kp)) // 位置增益必须是有限数，拒绝 NaN 和正负无穷。
    {
        return false;
    }
    if (config.yaw_angle_kp < 0.0f) // 本例不接受负位置增益，方向应通过电机与坐标约定处理。
    {
        return false;
    }
    if (!std::isfinite(config.yaw_speed_limit)) // 速度上限不能是 NaN 或无穷，否则限幅失去意义。
    {
        return false;
    }
    if (config.yaw_speed_limit <= 0.0f) // 速度上限必须大于零，才形成有效的对称输出限幅。
    {
        return false;
    }
    if (config.yaw_gyro_axis != GimbalGyroAxis::Z) // 本例仅实现 Z 轴 Yaw，不支持仅靠改枚举适配其他安装姿态。
    {
        return false;
    }
    bool valid_sign = false;
    if (config.yaw_gyro_sign == 1.0f) // 接受与约定一致的角速度反馈正方向。
    {
        valid_sign = true;
    }
    else if (config.yaw_gyro_sign == -1.0f) // 接受反向角速度反馈标定，只允许正一或负一。
    {
        valid_sign = true;
    }
    if (!valid_sign) // 方向符号不是正一或负一，配置无效。
    {
        return false;
    }
    Struct_DJIMotor_Init_Config motor_config{}; // 零初始化未使用的环参数，不能留未初始化字段。
    motor_config.hfdcan = config.yaw.bus;
    motor_config.can_id = 1; // GM6020 电机编号 1~7，不是达妙发送 ID 或 Master ID。
    motor_config.motor_type = Enum_DJIMotor_Type::GM6020;
    motor_config.control_mode = Enum_DJIMotor_Control_Mode::VOLTAGE; // 协议原始电压指令，需匹配电机端设置。
    motor_config.close_loop = DJI_MOTOR_SPEED_LOOP; // App 计算位置环，驱动只计算速度环。
    motor_config.outer_loop = DJI_MOTOR_SPEED_LOOP;
    motor_config.gear_ratio = 1.0f; // 本例假定 GM6020 直接驱动 Yaw。
    motor_config.reverse = config.yaw.reverse;
    motor_config.feedback_timeout_ms = 20; // 电机反馈超时由驱动判断，与 INS 的 10 ms 独立。
    motor_config.speed_pid.K_P = 100.0f; // 仅演示速度环配置，单位为协议指令/(rad/s)，必须实机整定。
    motor_config.speed_pid.K_I = 0.0f;
    motor_config.speed_pid.K_D = 0.0f;
    motor_config.speed_pid.Out_Max = 1000.0f; // 示例输出上限是协议原始值，不是 V、A 或 N·m。
    motor_config.speed_pid.D_T = 0.001f; // 与 1 kHz 更新周期一致。
    registered = yaw_motor.Init(motor_config);
    if (!registered) // 电机注册失败，不继续绑定发送组或许可输出。
    {
        return false;
    }
    group_registered = yaw_group.Init(&yaw_motor); // 所有共享此发送报文的电机必须先注册，再统一绑定。
    initialized = group_registered;
    return initialized;
}
```

</details>

<details>
<summary>8. Gimbal_GetStatus()：读取单轴状态（GM6020）</summary>

读取最近的状态快照；没有协议故障码时用 INS 有效性与 DJI online 判断故障。

```cpp
Enum_Gimbal_Status Gimbal_GetStatus(void)
{
    if (!initialized) // 应用尚未完成有效配置和设备注册，不能正常输出。
    {
        return Gimbal_Status_CONFIG_ERROR;
    }
    if (command.mode == GimbalMode::DISABLED) // 禁用命令优先于在线与就绪判断，状态报告为禁用。
    {
        return Gimbal_Status_DISABLE;
    }
    if (!ins_valid) // INS 未发布或已经超过时效，不能使用旧姿态继续闭环。
    {
        return Gimbal_Status_FAULT;
    }
    if (!snapshot.online) // DJI 反馈已过期或尚无反馈，报告故障；没有达妙协议故障码。
    {
        return Gimbal_Status_FAULT;
    }
    if (snapshot.ready) // 就绪只代表反馈与本地/协议许可满足，不保证参数已整定或目标已执行。
    {
        return Gimbal_Status_READY;
    }
    else
    {
        return Gimbal_Status_ENABLING;
    }
}
#endif
```

</details>

<details>
<summary>9. Gimbal_Update()：主逻辑入口（GM6020）</summary>

每周期读取输入后只调用一次 PrepareControl，再更新目标并调用 Control 计算位置 PID。yaw_group.Control(speed_rad_s) 同时设置速度目标、执行驱动速度 PID 并提交组报文。单独调用 yaw_motor.Control() 只组帧，不会提交 CAN。

```cpp
void Gimbal_Update(void)
{
    ins_valid = MessageCenter::INS_State_Topic.ReadFresh(ins, 10000U); // 10000 的单位是 us；失败后即使 ins 仍有旧值，也不得继续正常控制。
#if GIMBAL
    TopicSnapshot<GimbalCmd> message = MessageCenter::Gimbal_Command_Topic.ReadWithMeta(); // 一次取得命令与有效标志/序号；这里没有检查命令年龄，失联许可由 Input/RobotCmd 管理。
    if (message.valid) // 使用已发布命令；否则采用默认 DISABLED，避免沿用旧活动命令。
    {
        command = message.data;
    }
    else
    {
        GimbalCmd default_command; // 成员默认值使模式为 DISABLED，不沿用之前的活动命令。
        command = default_command;
    }
    snapshot = yaw_motor.GetMotionSnapshot(); // 读取驱动对新鲜度、协议状态与请求的判断；软件提交成功不能代替 ready。
    if (PrepareControl(message)) // 安全与恢复处理允许控制后，才更新目标并计算电机输出。
    {
        UpdateTarget(message);

        Control();
    }
#endif
    PublishFeedback(); // 每周期只调用一次，故障、等待和正常路径共享同一反馈分频。
}
```

</details>

### 开发后的检查顺序

先验证接口可编译、输入有效性、限幅和停机路径，再验证硬件。
至少覆盖初始化失败、INS 超过 10 ms、电机未就绪/离线、DISABLED、LOCK 切换、
IMU 新目标及恢复后旧目标不重放。检查命令静止和同值重新发布两种情况。
固件构建按所改板型执行：

```sh
cmake --preset GimbalBoard
cmake --build --preset GimbalBoard
# 同时支持单板时，打开其硬件控制路径再验证：
cmake --preset SingleBoard -DH7_APP_GIMBAL=ON
cmake --build --preset SingleBoard
```

SingleBoard 的 CMake 选项会留在本地缓存；需要恢复默认关闭时重新配置
`cmake --preset SingleBoard -DH7_APP_GIMBAL=OFF`。构建只证明代码集成成功，
实机仍需逐项验证下方内容。

## 验证

尚未完成板测：需要验证型号/量程、方向/零位、MIT 增益、Yaw 转矩环、机械限位、
CAN 满载、断线恢复、使能顺序及实际控制周期。

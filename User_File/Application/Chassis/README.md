# 四舵轮底盘 Application

当前实现是四个舵轮模块：4 台 M3508 行走电机、4 台 M3508 舵向电机。
Application 把底盘速度分解为轮速与舵角，DJI Device 执行电机闭环和分组提交。
这些参数是配置参考，机械零位、接线方向和 PID 尚需实车标定。

## 移植入口与框架接口

| 文件 | 移植或重新开发时负责什么 |
| --- | --- |
| [Chassis.h](Chassis.h) | 保留初始化与周期更新两个公开入口 |
| [Chassis_Config.h](Chassis_Config.h) | 轮半径、轮位置、舵角偏置、编号和控制增益 |
| [Chassis.cpp](Chassis.cpp) | 私有设备、运动学、输出组织、反馈估计 |
| [SingleBoard 任务](../../Task/Control_Task.cpp) | RobotCmd 更新之后调用底盘，1 kHz 调度 |
| [ChassisBoard 任务](../../Task/Control_Task_Chassis.cpp) | Transport 轮询之后调用底盘，1 kHz 调度 |
| [RobotCmd](../RobotCmd/RobotCmd.cpp) | 统一组织并发布命令，底盘命令每 10 ms 刷新 |
| [BoardConfig](../../../User_Config/Board/board_config.h) | 分配行走轮和舵向总线；接线变动核对对应板型实现 |
| [消息类型](../../System/MessageCenter/message_types.h) | 命令、反馈及模式的唯一数据契约 |
| [Transport 指南](../../System/Transport/README.md) | 双板命令和反馈的编码、解码、时效约定 |
| [根 CMake](../../../CMakeLists.txt) | 板型选择、新增源文件及 include 注册 |

### 对外函数与调用顺序

| 接口 | 调用时机 | 实现职责 |
| --- | --- | --- |
| `bool Chassis_Init(void)` | 所选 ControlTask 启动时仅调用一次 | 清空应用状态、注册电机、绑定发送组，返回是否成功 |
| `void Chassis_Update(void)` | 同一个 ControlTask 每 1 ms 调用 | 读取命令、快照，计算并提交输出，更新并分频发布反馈 |

SingleBoard 的顺序是输入更新 → RobotCmd → Gimbal → Chassis → Shoot。
ChassisBoard 的顺序是 `BoardTransport_Poll()` → `Chassis_Update()`，该板不运行 RobotCmd。
两份任务当前都忽略初始化返回值；应用内部 `initialized` 阻止失败后的正常控制。
需要启动诊断时在已有初始化位置处理返回值，不在周期里重复 Init。

SingleBoard 默认 `H7_APP_CHASSIS=OFF`；ChassisBoard 固定开启，GimbalBoard 不编译本应用。
`CHASSIS=0` 时仍保留两个公开入口，但不注册电机，反馈按 100 Hz 发布默认值。
不用另建控制任务，也不要在 CAN 接收中断中调用底盘更新。

### 命令与反馈

| 通道 | 来源与去向 | 字段及约束 |
| --- | --- | --- |
| `Chassis_Command_Topic` | SingleBoard 的 RobotCmd，或 ChassisBoard 的 Transport → 底盘 | vx/vy 为 m/s，wz 为 rad/s，100 ms 内有效 |
| `Chassis_Feedback_Topic` | 底盘 → 本地消费者/Transport | 估计 vx/vy/wz，八电机 online 与输出许可 enabled，100 Hz 发布 |

`ReadFresh()` 失败后使用默认 `ZERO_FORCE`，不继续使用 Topic 中的旧目标。
恰好 100 ms 的命令仍有效；发送者应定期刷新静止目标，不能只在数值变化时发布。
由已有 RobotCmd/Transport 保持通道写入职责，底盘不要自己发布自己的命令。

| 模式 | 当前底盘实际处理 |
| --- | --- |
| `ZERO_FORCE` | 两组请求失能并清零协议输出，不保持舵角闭环 |
| `NO_FOLLOW` | 直接使用命令 vx/vy/wz 计算四轮目标 |
| `FOLLOW_GIMBAL_YAW` | 同上，没有独立的云台相对角跟随计算 |
| `ROTATE` | 同上，旋转速度由命令 wz 给出，没有内置固定转速 |

模式名不代表算法已实现。当前源码只判断是否 ZERO_FORCE，其余模式走相同计算。
命令单位和数值有效性由输入/Transport 边界负责；应用没有额外有限值、模式范围或车速校验。

## 机构参数与控制链

行走轮：速度目标 rad/s → DJI 速度 PID → 协议原始电流指令。
舵向：累计角度目标 rad → 角度 PID → rad/s → 速度 PID → 协议指令。
当前不启用软件电流 PID，电流指令也不是安培或 N·m。

| 配置 | 含义与移植要求 |
| --- | --- |
| half_length / half_width | 轮位置的半长/半宽，单位 m，反解分母必须非零 |
| wheel_radius | 有效轮半径 m，用于轮速与线速度转换，必须为正 |
| steer_offset[4] | 各轮机械零位偏置 rad，必须与数组索引和实际轮模块一致 |
| motor_id[4] | DJI 编号 1～4，两组各自使用；两组必须有不冲突的总线/报文槽 |
| wheel_speed_pid / steer_speed_pid | 速度误差为 rad/s，输出为协议原始值，需重新整定 |
| steer_angle_pid | 角误差 rad，输出与积分限幅为 rad/s |
| feedback_alpha | 1 kHz 更新的平滑权重；0 不更新、1 不平滑，需按实际周期选取 |
| stop_speed | 近零轮速阈值 m/s，避免无运动需求时舵向跳变 |

轮索引 0～3 沿用源码数组；从旋转分量看，对应相对位置 `(x,y)` 为
`(+L,-W)`、`(-L,-W)`、`(-L,+W)`、`(+L,+W)`，这里采用 vx=vx_body−wz·y、vy=vy_body+wz·x 的数学约定。
这不自动证明实车前/左方向正确；接线、传动方向和正转轴仍需按安装核对。

目标角先加机械偏置，再把目标与当前累计角度的差折回 ±π。
超过 ±π/2 时补偿 π 并反转行走轮，让舵向走更短的路径。
近零轮速时行走目标为零、舵向保持当前角度；它与 ZERO_FORCE 失能不同。
增量编码器首次反馈不提供绝对机械零位，偏置数组不能代替寻零或已知上电姿态。

## 安全、掉线与反馈的实际边界

设备在线由 DJI 快照读取 Daemon 即时状态，Control/Send 不等待低频检查即可清零离线电机。
StatusTask 的 DJI ServiceAll 还会独立覆盖安全帧并清积分，即使 App 不再调用 Control/Send。
当前应用没有“一个模块掉线则八台电机全部停机”的聚合策略；其他在线电机可能继续控制。
若要增加全车许可、恢复状态机或集中 PrepareControl，应单独设计行为并验证，不把文档示例当作现有行为。

本应用没有云台那样的恢复捕获流程：命令仍新鲜时，恢复后的电机继续使用当前目标。
初始化失败后不进入组控制；已成功注册的电机保持驱动默认禁止输出，初始化不可反复调用。
组 Control 的返回值当前未处理；它表示软件提交与设备就绪结果，不能等同于 CAN 已发送或机械已执行。

反馈用轮角速度乘半径得到线速度，按舵角投影并反解车体速度。
没有积分位置、IMU 融合或滑移补偿，不是完整里程计。
`online` 表示八台电机反馈都在线；`enabled` 表示非 ZERO_FORCE 且读取的八台快照都 ready。
快照在使能请求之前获取，enabled 可能比本周期新请求滞后一周期；不要将它视为发送确认。
命令失效、反馈失效时，速度反馈也不会自动全部清零，消费者须同时检查 online/enabled。

## 重新开发时改哪些函数

| 内部对象/函数 | 职责与修改位置 |
| --- | --- |
| `ChassisContext` | 唯一私有设备所有者、命令和快照；保留静态生命周期 |
| `Chassis_MakePID()` | 机构参数转为驱动配置，保持 dt 与实际周期一致 |
| `Chassis_CalculateTargets()` | 四舵轮运动学及最短转向；换麦轮/差速模型主要从这里开始 |
| `Chassis_UpdateFeedback()` | 与前向模型配套的反馈反解及标志聚合，换模型时同步修改 |
| `Chassis_Init()` | 设备型号、编号、总线、环配置及发送组成员；先注册全部成员再绑定组 |
| `Chassis_Update()` | 新鲜度、输出许可、调度顺序与反馈分频；不在内部建立第二个命令发布者 |

只换参数时改配置与 BoardConfig；换模型时同时修改设备数量、前向运动学和反馈反解。
更换电机先核对型号、gear_ratio、协议输出量纲、组报文共享与反馈单位。
同一物理发送组不能被多个逻辑组分别绑定，不能只调用单电机 Control 而漏掉 Send。
外部反馈指针必须指向长期有效对象，不要绑定初始化函数的局部变量。

更改消息字段或模式时同步 Input/RobotCmd、应用消费者、反馈读取方；双板还需修改两端 Transport。
新增文件在 CMake 显式注册，检查 SingleBoard、ChassisBoard 的源文件选择。
周期内不能阻塞等待反馈、动态分配或输出阻塞日志。

## 内部代码例程

下面按函数折叠展示当前实现，注释解释判断的业务含义、物理单位和接口约束。
这些代码共用源码的 include、常量与私有 ctx；完整集成以 Chassis.cpp 为准，不能在另一文件重复注册设备。

<details>
<summary>ChassisContext：设备与状态</summary>

设备对象由底盘唯一持有，两个发送组各包含四台电机；数组索引贯穿运动学和反馈。

```cpp
namespace
{
struct ChassisContext
{
    Publisher<ChassisFeedback> feedback_publisher{MessageCenter::Chassis_Feedback_Topic};
    ChassisCmd command{};
    ChassisFeedback feedback{};
    uint8_t feedback_divider = 0U;
#if CHASSIS
    Class_DJIMotor wheel_motor[4];
    Class_DJIMotor steer_motor[4];
    Struct_DJIMotor_Motion_Snapshot wheel_snapshot[4];
    Struct_DJIMotor_Motion_Snapshot steer_snapshot[4];
    Class_DJIMotor_Group wheel_group;
    Class_DJIMotor_Group steer_group;
    bool initialized = false;
    int8_t wheel_direction[4] = {1, 1, 1, 1};
#endif
};

ChassisContext ctx;
}
```

</details>

<details>
<summary>Chassis_MakePID()：参数与周期</summary>

未使用参数零初始化，dt 固定为 1 ms。角度环与速度环的输出单位不同。

```cpp
static PID_InitTypeDef Chassis_MakePID(const ChassisPidConfig& config)
{
    PID_InitTypeDef pid{};
    pid.K_P = config.kp;
    pid.K_I = config.ki;
    pid.K_D = config.kd;
    pid.I_Out_Max = config.integral_limit;
    pid.Out_Max = config.output_limit;
    pid.D_T = 0.001f;
    return pid;
}
```

</details>

<details>
<summary>Chassis_CalculateTargets()：四舵轮目标</summary>

使用本周期舵角快照，目标使用累计 rad；最短转向必须与反转轮速配套。

```cpp
static void Chassis_CalculateTargets(float wheel_target_rad_s[4],
                                     float steer_target_rad[4])
{
    /* 四轮位置的旋转项为 ±wz·半宽/半长；符号按下方轮索引数组固定。
       物理前/左和正转方向必须由实车接线及坐标标定确认。 */
    const float vx_m_s = ctx.command.velocity_x_m_s;
    const float vy_m_s = ctx.command.velocity_y_m_s;
    const float wz_rad_s = ctx.command.angular_velocity_rad_s;
    const float wheel_vx[4] = {
        vx_m_s + wz_rad_s * kChassisConfig.half_width_m,
        vx_m_s + wz_rad_s * kChassisConfig.half_width_m,
        vx_m_s - wz_rad_s * kChassisConfig.half_width_m,
        vx_m_s - wz_rad_s * kChassisConfig.half_width_m,
    };
    const float wheel_vy[4] = {
        vy_m_s + wz_rad_s * kChassisConfig.half_length_m,
        vy_m_s - wz_rad_s * kChassisConfig.half_length_m,
        vy_m_s - wz_rad_s * kChassisConfig.half_length_m,
        vy_m_s + wz_rad_s * kChassisConfig.half_length_m,
    };

    for (uint8_t index = 0; index < 4; ++index)
    {
        const float velocity_m_s = std::sqrt(wheel_vx[index] * wheel_vx[index] +
                                             wheel_vy[index] * wheel_vy[index]);
        const float current_angle_rad =
            ctx.steer_snapshot[index].output_total_angle;
        if (velocity_m_s < kChassisConfig.stop_speed_m_s) // 近零轮速不确定方向，保持舵角，避免微小速度输入造成转向跳变。
        {
            // 近零轮速时保持当前舵角，避免 atan2 的方向随微小输入跳变。
            wheel_target_rad_s[index] = 0.0f;
            steer_target_rad[index] = current_angle_rad;
            continue;
        }

        const float target_angle_rad = std::atan2(wheel_vy[index], wheel_vx[index]) +
                                       kChassisConfig.steer_offset_rad[index];
        float difference_rad = std::remainder(target_angle_rad - current_angle_rad,
                                              2.0f * kPiRad);
        /* remainder 将误差压到 [-π, π]；超过 ±π/2 时舵角少转 π、轮速取反。 */
        if (difference_rad > kPiRad / 2.0f) // 正向舵角差超过 90°，少转 180°并反转行走轮可保持同一速度向量。
        {
            difference_rad -= kPiRad;
            ctx.wheel_direction[index] = -1;
        }
        else if (difference_rad < -kPiRad / 2.0f) // 负向舵角差超过 90°，同样通过转向补偿与轮速反转缩短转向路径。
        {
            difference_rad += kPiRad;
            ctx.wheel_direction[index] = -1;
        }
        else
        {
            ctx.wheel_direction[index] = 1;
        }

        steer_target_rad[index] = current_angle_rad + difference_rad;
        // 线速度除以轮半径得到输出轴 rad/s；后续闭环由 DJI 驱动的现有 PID 执行。
        wheel_target_rad_s[index] = (velocity_m_s / kChassisConfig.wheel_radius_m) *
                                    ctx.wheel_direction[index];
    }
}
```

</details>

<details>
<summary>Chassis_UpdateFeedback()：速度估计</summary>

减去机械偏置后计算轮方向，反馈平滑按控制周期更新；不是实测车体速度。

```cpp
static void Chassis_UpdateFeedback(void)
{
    float wheel_vx[4];
    float wheel_vy[4];
    bool online = true;
    bool ready = true;
    for (uint8_t index = 0; index < 4; ++index)
    {
        const float heading_rad =
            ctx.steer_snapshot[index].output_total_angle -
            kChassisConfig.steer_offset_rad[index];
        const float linear_speed_m_s =
            ctx.wheel_snapshot[index].output_speed *
            kChassisConfig.wheel_radius_m;
        wheel_vx[index] = linear_speed_m_s * std::cos(heading_rad);
        wheel_vy[index] = linear_speed_m_s * std::sin(heading_rad);
        online = online && ctx.wheel_snapshot[index].online &&
                 ctx.steer_snapshot[index].online;
        ready = ready && ctx.wheel_snapshot[index].ready &&
                ctx.steer_snapshot[index].ready;
    }

    const float vx = (wheel_vx[0] + wheel_vx[1] + wheel_vx[2] + wheel_vx[3]) * 0.25f;
    const float vy = (wheel_vy[0] + wheel_vy[1] + wheel_vy[2] + wheel_vy[3]) * 0.25f;
    const float wz_x = ((wheel_vx[0] - wheel_vx[2]) +
                        (wheel_vx[1] - wheel_vx[3])) /
                       (4.0f * kChassisConfig.half_width_m);
    const float wz_y = ((wheel_vy[0] - wheel_vy[1]) +
                        (wheel_vy[3] - wheel_vy[2])) /
                       (4.0f * kChassisConfig.half_length_m);

    // 一阶平滑 y += alpha * (x - y)，在 1 kHz 控制周期更新；100 Hz 仅是发布频率。
    // 初始输出沿用初始化时的零值，feedback_alpha 是每个控制周期的权重。
    ctx.feedback.velocity_x_m_s += kChassisConfig.feedback_alpha *
                                   (vx - ctx.feedback.velocity_x_m_s);
    ctx.feedback.velocity_y_m_s += kChassisConfig.feedback_alpha *
                                   (vy - ctx.feedback.velocity_y_m_s);
    ctx.feedback.angular_velocity_rad_s += kChassisConfig.feedback_alpha *
                                           (0.5f * (wz_x + wz_y) - ctx.feedback.angular_velocity_rad_s);
    // 快照在本周期使能请求之前读取，enabled 可能滞后一周期；不代表 CAN 已发送。
    ctx.feedback.enabled = ctx.command.mode != ChassisMode::ZERO_FORCE && ready;
    ctx.feedback.online = online;
}
```

</details>

<details>
<summary>Chassis_Init()：设备与发送组</summary>

依次尝试注册各电机，所有注册成功才绑定发送组。短路逻辑意味着初始化失败时不继续正常组绑定。

```cpp
bool Chassis_Init(void)
{
    ctx.command = {};
    ctx.feedback = {};
    ctx.feedback_divider = 0U;

#if CHASSIS
    Struct_DJIMotor_Init_Config wheel_config{};
    wheel_config.hfdcan = BoardConfig_Get().chassis_wheel_bus;
    wheel_config.motor_type = Enum_DJIMotor_Type::M3508;
    wheel_config.close_loop = DJI_MOTOR_SPEED_LOOP;
    wheel_config.outer_loop = DJI_MOTOR_SPEED_LOOP;
    // 速度环输入现为 rad/s；以下增益来源未标定，需实车重新整定。
    wheel_config.speed_pid = Chassis_MakePID(kChassisConfig.wheel_speed_pid);

    Struct_DJIMotor_Init_Config steer_config{};
    steer_config.hfdcan = BoardConfig_Get().chassis_steer_bus;
    steer_config.motor_type = Enum_DJIMotor_Type::M3508;
    steer_config.close_loop = DJI_MOTOR_ANGLE_LOOP | DJI_MOTOR_SPEED_LOOP;
    steer_config.outer_loop = DJI_MOTOR_ANGLE_LOOP;
    // 角度环输出为 rad/s：原 200/1000 deg/s 限幅作物理等效转换。
    // Kp/Ki 和舵轮速度环增益没有可信实车来源，启用前均需重新整定。
    steer_config.angle_pid = Chassis_MakePID(kChassisConfig.steer_angle_pid);
    steer_config.speed_pid = Chassis_MakePID(kChassisConfig.steer_speed_pid);

    bool initialized = true;
    for (uint8_t index = 0; index < 4; ++index)
    {
        wheel_config.can_id = kChassisConfig.motor_id[index];
        steer_config.can_id = kChassisConfig.motor_id[index];
        initialized = ctx.wheel_motor[index].Init(wheel_config) && initialized;
        initialized = ctx.steer_motor[index].Init(steer_config) && initialized;
    }
    initialized = initialized && ctx.wheel_group.Init(
                                     &ctx.wheel_motor[0], &ctx.wheel_motor[1],
                                     &ctx.wheel_motor[2], &ctx.wheel_motor[3]);
    initialized = initialized && ctx.steer_group.Init(
                                     &ctx.steer_motor[0], &ctx.steer_motor[1],
                                     &ctx.steer_motor[2], &ctx.steer_motor[3]);
    ctx.initialized = initialized;
    if (initialized) // 仅在全部电机与发送组初始化成功后提交零输出；不代表设备已经在线。
    {
        (void) ctx.wheel_group.RequestEnabled(false);
        (void) ctx.steer_group.RequestEnabled(false);
    }
    return initialized;
#else
    return true;
#endif
}
```

</details>

<details>
<summary>Chassis_Update()：完整更新入口</summary>

命令新鲜度与设备反馈时效分别维护；无效命令回到 ZERO_FORCE，反馈始终保持分频。

```cpp
void Chassis_Update(void)
{
    /* 仅在命令 Topic 仍新鲜时沿用目标；过期后使用默认 ZERO_FORCE 关闭输出。 */
    ChassisCmd command{};
    // 命令需存在且年龄不超过 100 ms；双板由 Transport 发布，本地由 RobotCmd 发布。
    if (MessageCenter::Chassis_Command_Topic.ReadFresh(
            command, CHASSIS_COMMAND_MAX_AGE_US))
    {
        ctx.command = command;
    }
    else
    {
        ctx.command = {};
    }

#if CHASSIS
    if (ctx.initialized) // 初始化失败时不访问正常组控制流程，反馈仍按周期发布。
    {
        for (uint8_t index = 0U; index < 4U; ++index)
        {
            ctx.wheel_snapshot[index] = ctx.wheel_motor[index].GetMotionSnapshot();
            ctx.steer_snapshot[index] = ctx.steer_motor[index].GetMotionSnapshot();
        }
        const bool enabled = ctx.command.mode != ChassisMode::ZERO_FORCE;
        (void) ctx.wheel_group.RequestEnabled(enabled);
        (void) ctx.steer_group.RequestEnabled(enabled);
        if (enabled) // 非 ZERO_FORCE 才计算运动目标；实际反馈超时保护由驱动逐电机执行。
        {
            float wheel_target_rad_s[4];
            float steer_target_rad[4];
            Chassis_CalculateTargets(wheel_target_rad_s, steer_target_rad);
            ctx.wheel_group.Control(wheel_target_rad_s[0], wheel_target_rad_s[1],
                                    wheel_target_rad_s[2], wheel_target_rad_s[3]);
            ctx.steer_group.Control(steer_target_rad[0], steer_target_rad[1],
                                    steer_target_rad[2], steer_target_rad[3]);
        }
        Chassis_UpdateFeedback();
    }
#endif

    /* 电机控制按 1 kHz 执行，反馈消息按 100 Hz 发布。 */
    ctx.feedback_divider++;
    if (ctx.feedback_divider >= 10U) // 1 kHz 下每十周期发布一次，反馈采样和平滑仍是 1 kHz。
    {
        ctx.feedback_divider = 0U;
        ctx.feedback_publisher.Publish(ctx.feedback);
    }
}
```

</details>

## 最简使用例程：生成直行或零力命令

<details>
<summary>展开：明确的 if/else 生成直行命令</summary>

此函数只构造命令，不新建 Topic 发布者，也不直接操作电机。
它可用于上层输入映射的教学参考；应在已有 Input/RobotCmd 仲裁路径中使用返回值，
保留遥控许可和来源选择。不要在底盘 Update 内调用它覆盖收到的真实命令。

```cpp
ChassisCmd MakeChassisExampleCommand(bool allow_motion)
{
    ChassisCmd command; // 默认字段为零、模式 ZERO_FORCE，异常时不沿用运动目标。
    if (allow_motion) // 上层已确认运动许可时，才构造直行需求。
    {
        command.mode = ChassisMode::NO_FOLLOW;
        command.velocity_x_m_s = 0.2f; // 示例线速度 m/s，实际“向前”方向须核对车体坐标。
        command.velocity_y_m_s = 0.0f;
        command.angular_velocity_rad_s = 0.0f;
    }
    else
    {
        command.mode = ChassisMode::ZERO_FORCE; // 零力停止，不维持主动舵角保持。
    }
    return command;
}
```

这不是控制环参数整定示例。应用收到新鲜命令后，现有运动学与驱动负责转换和执行。
若只测试行走速度目标为零但需要舵角保持，可在许可成立时使用 NO_FOLLOW 加零速度，
其行为与 ZERO_FORCE 不同。目标数值不变时上层也要每 10 ms 刷新，以满足 100 ms 时效。

</details>

## 四麦轮底盘完整实现例程

下面按函数给出基于当前框架的完整 `Chassis.cpp`，使用四台达妙 DM3519、现有 Topic 和达妙速度模式。各段依次组合即可组成文件；只作教学示例，生产代码仍为四舵轮。

混合符号参考老步兵，采用 45°麦轮的几何转换：车体 m/s 与 rad/s 先换算为轮轴 rad/s。
轮序与电机逻辑正向需满足代码中的混合矩阵；半径和半长/半宽按实际机构配置；本例假设电机输出轴直接驱动轮轴。
沿用 Chassis_Config 的几何参数，不使用 DJI PID、舵角偏置或舵向设备。
电机端提前设为速度模式，配置节点 ID 1～4、反馈 ID 0x60～0x63；反馈量程 12.5 rad、30 rad/s、10 N·m 仅为示例，必须与电机端一致。
`Init(...SPEED...)` 不会自动切换电机端模式；速度闭环由电机端执行。
例程任一轮掉线时撤销四轮输出，恢复且命令仍新鲜时继续当前目标；未加入加速度规划。

<details>
<summary>1. 头文件与私有状态</summary>

```cpp
#include "Chassis.h"
#include "Chassis_Config.h"
#include "board_config.h"
#include "message_center.h"
#include <cmath>
#if CHASSIS
#include "dmmotor.h"
#endif

namespace
{
ChassisCmd command;
ChassisFeedback feedback;
uint8_t feedback_divider = 0U;
#if CHASSIS
Class_DMMotor wheel_motor[4];
Struct_DMMotor_Snapshot snapshot[4];
bool registered[4] = {false, false, false, false};
bool initialized = false;
const float wheel_speed_max_rad_s = 30.0f; // 轮轴速度上限，需与电机量程和机构能力匹配。
const uint8_t motor_id[4] = {1U, 2U, 3U, 4U};
const uint16_t feedback_id[4] = {0x60U, 0x61U, 0x62U, 0x63U};
const bool motor_reverse[4] = {false, false, false, false}; // 按混合矩阵核对每个轮子的逻辑正向。
```

</details>

<details>
<summary>2. PrepareControl()：命令时效与四轮掉线保护</summary>

```cpp
bool PrepareControl(bool command_valid)
{
    bool safe = true;
    if (!initialized) // 设备初始化失败时，不进行正常控制。
    {
        safe = false;
    }
    if (!command_valid) // 命令未发布或超过 100 ms，禁止沿用旧目标。
    {
        safe = false;
    }
    if (command.mode == ChassisMode::ZERO_FORCE) // 零力模式停止主动输出。
    {
        safe = false;
    }
    if (!std::isfinite(command.velocity_x_m_s)) // 无效 X 速度不能进入轮速混合。
    {
        safe = false;
    }
    if (!std::isfinite(command.velocity_y_m_s)) // 无效 Y 速度不能进入轮速混合。
    {
        safe = false;
    }
    if (!std::isfinite(command.angular_velocity_rad_s)) // 无效旋转速度不能进入轮速混合。
    {
        safe = false;
    }
    for (int index = 0; index < 4; index++)
    {
        snapshot[index] = wheel_motor[index].GetFeedbackSnapshot();
        if (!snapshot[index].online || snapshot[index].fault) // 任一轮掉线或协议故障即停四轮。
        {
            safe = false;
        }
    }
    if (!safe) // 所有禁用原因在这里统一撤销输出许可。
    {
        for (int index = 0; index < 4; index++)
        {
            if (registered[index]) // 部分初始化失败时也撤销已注册电机的输出许可。
            {
                (void) wheel_motor[index].RequestEnabled(false);
            }
        }
        return false;
    }
    bool ready = true;
    for (int index = 0; index < 4; index++)
    {
        (void) wheel_motor[index].RequestEnabled(true); // 只表达期望使能；协议确认由反馈判断。
        snapshot[index] = wheel_motor[index].GetFeedbackSnapshot();
        if (!snapshot[index].ready) // 等待全部电机确认使能，不阻塞，也不提前提交运动目标。
        {
            ready = false;
        }
    }
    if (!ready) // 部分轮已使能时维持零速度，等待其余轮就绪。
    {
        for (int index = 0; index < 4; index++)
        {
            (void) wheel_motor[index].SetSpeed(0.0f);
        }
        return false;
    }
    return true;
}
```

</details>

<details>
<summary>3. Control()：四麦轮解算和速度提交</summary>

```cpp
void Control()
{
    const float radius_m = kChassisConfig.wheel_radius_m;
    const float lever_m = kChassisConfig.half_length_m + kChassisConfig.half_width_m;
    float x = command.velocity_x_m_s / radius_m; // 平移 m/s 转为轮速贡献 rad/s。
    float y = command.velocity_y_m_s / radius_m;
    float w = command.angular_velocity_rad_s * lever_m / radius_m; // 旋转贡献包含轮位置与半径。
    float wheel_speed[4];
    wheel_speed[0] = y + x + w;
    wheel_speed[1] = y - x + w;
    wheel_speed[2] = -y - x + w;
    wheel_speed[3] = x - y + w;
    for (int index = 0; index < 4; index++)
    {
        if (wheel_speed[index] > wheel_speed_max_rad_s) // 正向轮速超限时裁剪。
        {
            wheel_speed[index] = wheel_speed_max_rad_s;
        }
        else if (wheel_speed[index] < -wheel_speed_max_rad_s) // 反向轮速超限时保留方向并裁剪。
        {
            wheel_speed[index] = -wheel_speed_max_rad_s;
        }
    }
    bool submitted = true;
    for (int index = 0; index < 4; index++)
    {
        if (!wheel_motor[index].SetSpeed(wheel_speed[index])) // 任一目标未被软件周期槽接受，本周期撤销四轮许可。
        {
            submitted = false;
        }
    }
    if (!submitted) // 提交不是执行确认；失败时用失能路径覆盖已有运动目标。
    {
        for (int index = 0; index < 4; index++)
        {
            (void) wheel_motor[index].RequestEnabled(false);
        }
    }
}
#endif
```

</details>

<details>
<summary>4. PublishFeedback()：配套速度反解</summary>

```cpp
void PublishFeedback(bool allowed)
{
    ChassisFeedback next;
#if CHASSIS
    bool online = true;
    bool ready = true;
    float q[4];
    for (int index = 0; index < 4; index++)
    {
        snapshot[index] = wheel_motor[index].GetFeedbackSnapshot();
        q[index] = snapshot[index].feedback.velocity; // 电机反馈 rad/s，本例假设电机输出轴直接驱动轮轴。
        if (!snapshot[index].online) // 反馈反解只使用全部在线的四轮数据。
        {
            online = false;
        }
        if (!snapshot[index].ready) // 任一轮不 ready，整车反馈不宣称已使能。
        {
            ready = false;
        }
    }
    next.online = online;
    next.enabled = false;
    if (allowed) // 运行许可与设备 ready 必须同时成立。
    {
        next.enabled = ready;
    }
    if (online) // 用与 Control 配套的逆矩阵估算车体速度，不能套用四舵轮反解。
    {
        const float radius_m = kChassisConfig.wheel_radius_m;
        const float lever_m = kChassisConfig.half_length_m + kChassisConfig.half_width_m;
        next.velocity_x_m_s = (q[0] - q[1] - q[2] + q[3]) * radius_m * 0.25f;
        next.velocity_y_m_s = (q[0] + q[1] - q[2] - q[3]) * radius_m * 0.25f;
        next.angular_velocity_rad_s = (q[0] + q[1] + q[2] + q[3]) * radius_m / (4.0f * lever_m);
    }
#endif
    feedback = next; // 不在线时速度保持默认零，但消费者仍必须检查 online。
    feedback_divider = feedback_divider + 1U;
    if (feedback_divider >= 10U) // 1 kHz 下每 10 ms 发布一次。
    {
        feedback_divider = 0U;
        MessageCenter::Chassis_Feedback_Topic.Publish(feedback);
    }
}
} // namespace
```

</details>

<details>
<summary>5. Chassis_Init()：四台 DM3519 注册</summary>

```cpp
bool Chassis_Init(void)
{
#if CHASSIS
    if (!std::isfinite(kChassisConfig.wheel_radius_m)) // 半径用于除法，必须为有限值。
    {
        return false;
    }
    if (kChassisConfig.wheel_radius_m <= 0.0f) // 半径必须为正，不能带入零或负几何尺度。
    {
        return false;
    }
    const float lever_m = kChassisConfig.half_length_m + kChassisConfig.half_width_m;
    if (!std::isfinite(lever_m)) // 反馈旋转反解要求有限的几何杠杆臂。
    {
        return false;
    }
    if (lever_m <= 0.0f) // 杠杆臂用于反解分母，必须为正。
    {
        return false;
    }
    initialized = false;
    for (int index = 0; index < 4; index++)
    {
        registered[index] = wheel_motor[index].Init(
            BoardConfig_Get().chassis_wheel_bus, motor_id[index], feedback_id[index],
            Enum_DMMotor_Mode::SPEED, motor_reverse[index], 12.5f, 30.0f, 10.0f); // 协议反馈量程须与各 DM3519 电机端设置一致。
        if (!registered[index]) // 注册失败时停止已经注册的成员。
        {
            for (int previous = 0; previous < index; previous++)
            {
                (void) wheel_motor[previous].RequestEnabled(false);
            }
            return false;
        }
        (void) wheel_motor[index].RequestEnabled(false);
    }
    initialized = true;
    return true;
#else
    return true;
#endif
}
```

</details>

<details>
<summary>6. Chassis_Update()：主逻辑入口</summary>

```cpp
void Chassis_Update(void)
{
    ChassisCmd received;
    bool command_valid = MessageCenter::Chassis_Command_Topic.ReadFresh(received, 100000U); // 单位 us，上层持续刷新目标。
    if (command_valid) // 使用本周期读到的新鲜命令。
    {
        command = received;
    }
    else
    {
        ChassisCmd stopped;
        command = stopped;
    }
    bool allowed = false;
#if CHASSIS
    allowed = PrepareControl(command_valid);
    if (allowed) // 通过集中运行许可后，主逻辑只负责轮速解算与提交。
    {
        Control();
    }
#endif
    PublishFeedback(allowed);
}
```

</details>

仍由现有 ControlTask 每 1 ms 调用，命令每 10 ms 刷新，反馈 100 Hz 发布。
需要实际采用此例程时，核对轮序、接线和电机端速度环参数；前向裁剪会改变组合比例，反馈是轮速运动学估计。
达妙每台电机独立提交目标，节点 ID、速度模式控制 ID 和反馈 ID 不得与其他设备冲突。
现有 100 Hz StatusTask 必须继续调用 `Class_DMMotor::ServiceAll()`，负责使能与安全目标补交。
若用于 ChassisBoard，需在根 CMake 为该板型加入 `dmmotor.cpp`，并启用 `H7_HAS_DM_MOTOR`，当前 ChassisBoard 默认不编译达妙驱动。

## 移植与验证顺序

1. 核对板型、两组总线、电机编号和报文槽；同总线上两组 M3508 使用相同编号会冲突。
2. 核对轮索引、传动比、轮半径和几何半长/半宽，确认反馈单位为输出轴 rad/rad/s。
3. 建立舵向绝对零位或明确的上电姿态，再设置偏置；配置偏置不等于完成寻零。
4. 单独核对行走轮正向和舵角正向，再验证直行、横移、旋转及 ±π/2 最短转向切换。
5. 验证命令过期、ZERO_FORCE、单电机掉线、恢复及反馈标志，确认当前逐电机保护是否满足整车要求。

ChassisBoard 构建：

```sh
cmake --preset ChassisBoard
cmake --build --preset ChassisBoard
```

单板需要显式打开硬件路径：

```sh
cmake --preset SingleBoard -DH7_APP_CHASSIS=ON
cmake --build --preset SingleBoard
```

本地 CMake 缓存会保存选项，需要恢复默认时配置 `-DH7_APP_CHASSIS=OFF`。
构建成功不能证明方向、零位、PID、轮地接触和全车故障策略已经验证。
实机尚需检查 CAN 负载、8 电机反馈时效、限幅、轮胎滑移和实际控制周期。

相关约定见 [Application 指南](../README.md)、
[Message Center](../../System/MessageCenter/README.md) 与
[DJI 驱动](../../Device/Peripheral/Motor/DJImotor/dji_motor.md)。

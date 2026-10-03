# RobotCmd 命令组织与分发指南

RobotCmd 拥有云台、底盘和发射的缓存命令，统一接收输入仲裁结果并通过 Output 发布。
它不访问电机、IMU、HAL 或 CAN 设备；机构算法由各 Application 执行。
本指南重点说明新增输入、重新开发命令逻辑以及与 [Shoot](../Shoot/README.md) 的衔接。

## 文件和生命周期

| 文件/接口 | 职责 |
| --- | --- |
| [RobotCmd.h](RobotCmd.h) | 初始化、更新、命令设置、射击事件与反馈读取接口 |
| [RobotCmd.cpp](RobotCmd.cpp) | 静态缓存、变化标志、输入许可与发布分频 |
| [message_types.h](../../System/MessageCenter/message_types.h) | 唯一业务消息类型及 SI 字段定义 |
| [InputState](../Input/input_state.h) | Remote/VTM/Keyboard 和 Vision 的输入快照 |
| [SourceArbitration](../Input/source_arbitration.h) | 来源许可、新鲜度、合法性与视觉目标仲裁 |
| [Output](../../System/MessageCenter/output.h) | 无动态分配的输出句柄，具体对象决定本地或远端发布 |

ControlTask 启动时绑定三个输出并调用 RobotCmd_Init，随后初始化输入和机构。
每 1 ms 的顺序为输入更新 → RobotCmd_Update → 各机构 Update。
SingleBoard 使用三个静态 LocalPublisher；GimbalBoard 的底盘输出使用静态 RemotePublisher，
云台与 Shoot 仍为本地 Topic；ChassisBoard 消费远端底盘命令，不运行 RobotCmd。
Output 不拥有绑定对象，发布器生命周期必须覆盖任务；不能绑定初始化函数的临时局部对象。

Init 要求三个输出全部已绑定，否则返回 false，原绑定与缓存保持不变。
成功后底盘为 ZERO_FORCE、Shoot 为 OFF，云台为 LOCK；云台和发射标记待发布。
内部 Input_Armed 初始为 true，首次 Update 的仲裁决定实际许可；无有效输入时立即改为
云台 DISABLED、底盘 ZERO_FORCE、Shoot OFF。不能把 Init 成功当作遥控解锁成功。

## 输入仲裁与命令发布

```text
设备/协议 → 输入适配 → InputState
  → SourceArbitration_Resolve → InputDecision
  → RobotCmd：许可边沿、来源切换、缓存、发布
  → Gimbal / Chassis / Shoot → Device
```

仲裁要求 Remote 在 50 ms 内有效，即使选择 VTM/Keyboard 也保留 Remote 安全许可。
选中来源必须有不早于切源时刻的新鲜合法命令，Remote 门限 50 ms，其他来源 100 ms；
失效后返回安全结果，不自动回退。Remote 的解锁去抖由输入适配维护，RobotCmd 不重复处理原始遥控帧。
允许 Vision 时有效目标覆盖云台角目标；Vision 失效使用 LOCK，Remote 失效仍整体失去许可。
详细输入接入规则见 [Application 指南](../README.md)。

| 输出 | 当前发布策略 | 对接时注意 |
| --- | --- | --- |
| GimbalCmd | 字段变化或被 SetGimbal 标记时发布 | 序号可用于恢复后识别新目标；不能随意按周期重复发布 |
| ChassisCmd | 每 10 个控制周期，安全边沿/来源切换可提前 | 为底盘 100 ms 命令时效与远端接收提供刷新 |
| ShootCmd | 字段变化或被 SetShoot 标记时发布 | Shoot 缓存持续状态，当前没有 Topic 年龄检查 |
| ShootEvent | 通过容量 8 FIFO 独立提交 | 单发/三连发不可改为可覆盖的 Topic 状态 |

GimbalChanged 与 ShootChanged 显式比较所有现有字段。扩展消息时必须补充变化判断，
否则新字段单独变化可能不触发发布。Chassis 每周期更新缓存，但普通 SetChassis 不设置 dirty，
其命令由分频刷新，调用 SetChassis 不代表当场已发布。

失去输入许可的边沿装载默认安全命令，标记三个通道立即发布，并清空发射队列。
许可已关闭时 Set 接口拒绝新目标；恢复后由当前仲裁结果重新装载。
来源改变且仲裁允许控制时，底盘立即刷新并清除前一来源积压事件；不会混用两份输入。
当前事件清队列使用 Pop 到空，调用约定应维持 ControlTask 中的唯一生产流程；不要从另一任务持续并发灌入事件。

## 公共接口与调用边界

| 接口 | 接受条件与结果 |
| --- | --- |
| RobotCmd_Init | 三个 Output 已绑定；只负责命令状态，不初始化电机 |
| RobotCmd_Update | 每 1 ms 调用，仲裁结果会覆盖 Set 接口缓存 |
| RobotCmd_SetGimbal | Input_Armed 时缓存并置 dirty；未许可则拒绝 |
| RobotCmd_SetChassis | Input_Armed 时缓存，等待周期发布 |
| RobotCmd_SetShoot | Input_Armed 时缓存持续状态并置 dirty |
| RobotCmd_PushShootEvent | Input_Armed 且 FIFO 有空间时返回 true；未许可或满返回 false |
| RobotCmd_Get*Feedback | Topic 存在且最近 100 ms 发布返回 true，失败不修改输出对象 |

缓存、dirty 和 Input_Armed 无同步保护，Init/Update/Set/Push 按控制任务单一上下文使用，
不可从 UART ISR 或其他任务并发调用。协议回调应先交给输入适配/同步快照。
三个反馈 getter 可在任务读取，返回 true 仅证明消息新鲜；仍需检查各反馈的 online、enabled，
云台还需检查 ins_valid。反馈新鲜不等于设备在线，Shoot.enabled 也不表示达速或完成发射。

## 与 Shoot 衔接：状态和动作分开

持续目标使用 ShootCmd：总开关、摩擦轮、拨弹模式以及速度/射速。
按钮边沿的一次动作使用 ShootEvent：ShootOnce 或 ShootTriple。
RobotCmd_PushShootEvent 只检查输入许可与队列容量，不检查 Shoot 总开关、达速或设备 ready。
成功表示逻辑动作入队；Shoot 在 ON+STOP 路径消费，在 OFF 路径清除，BURST/REVERSE 留在队列。

若需要“达速后才能单发”，应明确 Shoot 的消费条件与动作状态，不能把成功 Push 当作发射完成。
事件接口没有自动绑定任意输入按钮，新增按钮需在输入流程检测边沿并在许可明确后提交一次。
安全关闭或来源切换会清除旧事件，因此事件提交时机须与本周期仲裁顺序协调。

<details>
<summary>例程：读取发射反馈并判断可用性</summary>

```cpp
ShootFeedback feedback;
bool fresh = RobotCmd_GetShootFeedback(feedback);
if (!fresh) // 最近 100 ms 没有应用反馈，不能使用旧输出对象判断设备状态。
{
    return;
}
if (!feedback.online) // 应用消息新鲜，但至少一台电机没有新鲜设备反馈。
{
    return;
}
if (!feedback.enabled) // 总开关未开或至少一台电机未 ready；不是摩擦轮达速判据。
{
    return;
}
// 此处可读取摩擦轮 rad/s 和拨弹累计 rad；达速判据应另行定义。
```

</details>

## 重新开发或接入新输入

1. 在 InputState 提交入口转换协议单位，保存有效性与时间戳，不直接调用电机或发布应用 Topic。
2. 在 SourceArbitration 定义选源与合法性策略，保持安全许可和切源新命令要求。
3. RobotCmd 保持唯一命令发布权；修改消息字段时同步 GimbalChanged/ShootChanged、生产者和消费者。
4. Application 处理机构目标与状态机；硬件绑定归 BoardConfig，PID/机械参数归模块配置。
5. 涉及远端消息时同步 Transport 编码与解码，并核对固定发布周期和接收时效。

保留 Set 接口兼容已有调用方，但不要另建任务反复 Set 作为新输入通道：Update 会再次采用仲裁结果。
需独立生成命令的业务应纳入明确的仲裁/命令所有权设计，而不是绕过 Input_Armed。

## 按函数阅读当前实现

下面逐个折叠展示生产函数。函数注释说明职责，条件注释说明业务意义。

<details>
<summary>GimbalChanged()</summary>

```cpp
static bool GimbalChanged(const GimbalCmd& next)
{
    return Gimbal_Command.mode != next.mode ||
           Gimbal_Command.yaw_angle_rad != next.yaw_angle_rad ||
           Gimbal_Command.pitch_angle_rad != next.pitch_angle_rad ||
           Gimbal_Command.yaw_speed_rad_s != next.yaw_speed_rad_s ||
           Gimbal_Command.pitch_speed_rad_s != next.pitch_speed_rad_s;
}
```

</details>

<details>
<summary>ShootChanged()</summary>

```cpp
static bool ShootChanged(const ShootCmd& next)
{
    return Shoot_Command.shoot_mode != next.shoot_mode ||
           Shoot_Command.friction_mode != next.friction_mode ||
           Shoot_Command.loader_mode != next.loader_mode ||
           Shoot_Command.friction_speed_rad_s != next.friction_speed_rad_s ||
           Shoot_Command.loader_speed_rad_s != next.loader_speed_rad_s ||
           Shoot_Command.shoot_rate_hz != next.shoot_rate_hz;
}
```

</details>

<details>
<summary>RobotCmd_Init()</summary>

```cpp
bool RobotCmd_Init(Output<GimbalCmd> gimbal_output,
                   Output<ChassisCmd> chassis_output,
                   Output<ShootCmd> shoot_output)
{
    // 三个通道均需绑定，避免未绑定 Publish 静默丢弃命令。
    if (!gimbal_output.IsBound() || !chassis_output.IsBound() ||
        !shoot_output.IsBound())
    {
        return false;
    }
    Gimbal_Command_Output = gimbal_output;
    Chassis_Command_Output = chassis_output;
    Shoot_Command_Output = shoot_output;
    Gimbal_Command = {};
    Chassis_Command = {};
    Shoot_Command = {};
    /*
     * RM 安全启动默认值：云台就绪后捕获并保持当前姿态；底盘保持零力矩，
     * 发射机构保持关闭。
     */
    Gimbal_Command.mode = GimbalMode::LOCK;
    Gimbal_Command_Dirty = true;
    Shoot_Command_Dirty = true;
    Chassis_Command_Dirty = false;
    Input_Armed = true;
    RobotCmd_Chassis_Publish_Divider = 0U;
    Last_Input_Source = InputSource::Remote;
    return true;
}
```

</details>

<details>
<summary>RobotCmd_Update()</summary>

```cpp
void RobotCmd_Update(void)
{
    const InputDecision decision = SourceArbitration_Resolve(InputState_Read());
    const bool source_changed = decision.source != Last_Input_Source;
    RobotCmd_SetInputArmed(decision.armed);
    if (decision.armed) // 只有选中来源通过新鲜度、合法性和 Remote 安全许可才接受运动目标。
    {
        if (source_changed) // 来源改变时立即刷新底盘，并丢弃前一来源积压的射击请求。
        {
            Chassis_Command_Dirty = true;
            ShootEvent discarded{};
            // 切源清除旧动作；同一 ControlTask 拥有事件生产流程。
            while (MessageCenter::Shoot_Event_Queue.Pop(discarded))
            {
            }
        }
        if (GimbalChanged(decision.gimbal)) // 模式或目标变化才产生新的云台命令序号。
        {
            RobotCmd_SetGimbal(decision.gimbal);
        }
        RobotCmd_SetChassis(decision.chassis);
        if (ShootChanged(decision.shoot)) // 发射持续状态变化才发布，事件独立排队。
        {
            RobotCmd_SetShoot(decision.shoot);
        }
    }
    Last_Input_Source = decision.source;

    const bool chassis_publish_due = RobotCmd_Chassis_Publish_Divider == 0U;
    RobotCmd_Chassis_Publish_Divider = (RobotCmd_Chassis_Publish_Divider + 1U) % 10U;

    /* 云台和发射按变化发布，底盘命令固定周期刷新。 */
    if (Gimbal_Command_Dirty) // 缓存有待发布的云台目标，包括失联安全目标。
    {
        Gimbal_Command_Output.Publish(Gimbal_Command);
        Gimbal_Command_Dirty = false;
    }
    // 固定刷新为底盘本地或远端接收方提供命令时效。
    if (chassis_publish_due || Chassis_Command_Dirty) // 周期到达或需立即发布安全/切源目标。
    {
        Chassis_Command_Output.Publish(Chassis_Command);
        Chassis_Command_Dirty = false;
    }
    if (Shoot_Command_Dirty) // 缓存有待发布的发射状态，发布后清除标志。
    {
        Shoot_Command_Output.Publish(Shoot_Command);
        Shoot_Command_Dirty = false;
    }
}
```

</details>

<details>
<summary>RobotCmd_SetInputArmed()</summary>

```cpp
static void RobotCmd_SetInputArmed(bool armed)
{
    if (armed == Input_Armed) // 许可未变化，避免每周期重复重置命令和队列。
    {
        return;
    }
    Input_Armed = armed;
    if (!armed) // 失去许可的边沿立即将三个模块目标改为默认安全状态。
    {
        Gimbal_Command = {};
        Chassis_Command = {};
        Shoot_Command = {};
        Gimbal_Command_Dirty = true;
        Chassis_Command_Dirty = true;
        Shoot_Command_Dirty = true;
        ShootEvent discarded{};
        while (MessageCenter::Shoot_Event_Queue.Pop(discarded))
        {
        }
    }
}
```

</details>

<details>
<summary>RobotCmd_SetGimbal()</summary>

```cpp
void RobotCmd_SetGimbal(const GimbalCmd& command)
{
    if (!Input_Armed) // 输入许可关闭时拒绝外部设置，不能覆盖失联安全目标。
        return;
    Gimbal_Command = command;
    Gimbal_Command_Dirty = true;
}
```

</details>

<details>
<summary>RobotCmd_SetChassis()</summary>

```cpp
void RobotCmd_SetChassis(const ChassisCmd& command)
{
    if (!Input_Armed) // 输入许可关闭时拒绝外部设置，不能覆盖失联安全目标。
        return;
    Chassis_Command = command;
}
```

</details>

<details>
<summary>RobotCmd_SetShoot()</summary>

```cpp
void RobotCmd_SetShoot(const ShootCmd& command)
{
    if (!Input_Armed) // 输入许可关闭时拒绝外部设置，不能覆盖失联安全目标。
        return;
    Shoot_Command = command;
    Shoot_Command_Dirty = true;
}
```

</details>

<details>
<summary>RobotCmd_PushShootEvent()</summary>

```cpp
bool RobotCmd_PushShootEvent(const ShootEvent& event)
{
    return Input_Armed && MessageCenter::Shoot_Event_Queue.Push(event);
}
```

</details>

<details>
<summary>RobotCmd_GetGimbalFeedback()</summary>

```cpp
bool RobotCmd_GetGimbalFeedback(GimbalFeedback& feedback)
{
    return MessageCenter::Gimbal_Feedback_Topic.ReadFresh(feedback, FEEDBACK_MAX_AGE_US);
}
```

</details>

<details>
<summary>RobotCmd_GetChassisFeedback()</summary>

```cpp
bool RobotCmd_GetChassisFeedback(ChassisFeedback& feedback)
{
    return MessageCenter::Chassis_Feedback_Topic.ReadFresh(feedback, FEEDBACK_MAX_AGE_US);
}
```

</details>

<details>
<summary>RobotCmd_GetShootFeedback()</summary>

```cpp
bool RobotCmd_GetShootFeedback(ShootFeedback& feedback)
{
    return MessageCenter::Shoot_Feedback_Topic.ReadFresh(feedback, FEEDBACK_MAX_AGE_US);
}
```

</details>

## 验证重点

核对未绑定 Init 的失败路径、首次无效输入、安全许可丢失/恢复、来源切换清事件，
以及不变字段不发布、底盘固定周期刷新、队列满返回 false 和反馈过期不覆盖输出。
注释/文档修改检查差异与链接；业务变更覆盖 SingleBoard、GimbalBoard 和 ChassisBoard 的集成构建。
实机仍需确认输入掉线、CAN 通信和各机构实际停止行为。

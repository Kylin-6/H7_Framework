/**
 * @file RobotCmd.cpp
 * @brief 机器人命令所有权与分发中心，参考 Meta-Embedded-NG 设计。
 * @details
 * RobotCmd 是各 Application 命令的唯一发布者，按需读取各模块反馈。它只负责
 * 命令组织与模块间通信，不直接访问电机、CAN 或 IMU 设备。
 */

#include "RobotCmd.h"
#include "message_center.h"
#include "source_arbitration.h"

static Output<GimbalCmd> Gimbal_Command_Output;
static Output<ChassisCmd> Chassis_Command_Output;
static Output<ShootCmd> Shoot_Command_Output;
namespace
{
constexpr uint64_t FEEDBACK_MAX_AGE_US = 100000U; // 反馈消息时效 100 ms，不代替设备在线判断。
}

static GimbalCmd Gimbal_Command;
static ChassisCmd Chassis_Command;
static ShootCmd Shoot_Command;

/* 云台和发射按变化发布；底盘命令按 10 ms 刷新以提供失联时效。 */
static bool Gimbal_Command_Dirty;
static bool Shoot_Command_Dirty;
static bool Chassis_Command_Dirty;
static bool Input_Armed;
static uint8_t RobotCmd_Chassis_Publish_Divider;
static InputSource Last_Input_Source;
static void RobotCmd_SetInputArmed(bool armed);

/**
 * @brief 比较云台模式、角目标和速度前馈，决定是否发布新序号。
 * @note 对比缓存字段，不检测消息年龄或设备状态。
 */
static bool GimbalChanged(const GimbalCmd& next)
{
    return Gimbal_Command.mode != next.mode ||
           Gimbal_Command.yaw_angle_rad != next.yaw_angle_rad ||
           Gimbal_Command.pitch_angle_rad != next.pitch_angle_rad ||
           Gimbal_Command.yaw_speed_rad_s != next.yaw_speed_rad_s ||
           Gimbal_Command.pitch_speed_rad_s != next.pitch_speed_rad_s;
}

/**
 * @brief 比较发射总开关、子模式和持续目标，决定是否发布新序号。
 * @note 离散射击动作独立进入 FIFO，不参与该比较。
 */
static bool ShootChanged(const ShootCmd& next)
{
    return Shoot_Command.shoot_mode != next.shoot_mode ||
           Shoot_Command.friction_mode != next.friction_mode ||
           Shoot_Command.loader_mode != next.loader_mode ||
           Shoot_Command.friction_speed_rad_s != next.friction_speed_rad_s ||
           Shoot_Command.loader_speed_rad_s != next.loader_speed_rad_s ||
           Shoot_Command.shoot_rate_hz != next.shoot_rate_hz;
}

/**
 * @brief 绑定三个输出句柄并装载启动默认命令。
 * @return 任一输出未绑定返回 false，且不修改已有绑定与缓存。
 * @note 输出绑定对象须保持静态生命周期；启动 LOCK 在首次仲裁失效时会被 DISABLED 覆盖。
 */
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

/**
 * @brief 读取 InputState 仲裁结果，处理许可与来源切换，并按策略发布三类命令。
 * @note 由 ControlTask 每 1 ms 在输入更新后、各机构更新前调用。
 *       云台/发射按变化发布，底盘每 10 周期或安全/来源切换时刷新。
 */
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

/**
 * @brief 处理输入运行许可的边沿；失去许可时装载安全命令并清除待处理发射事件。
 * @note 相同许可不重复重置缓存；恢复时由本周期仲裁结果重新提供目标。
 */
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

/**
 * @brief 缓存云台命令并标记待发布，仅在输入许可成立时接受。
 * @note 控制任务运行后仲裁会覆盖缓存；新增输入优先接入 InputState。
 */
void RobotCmd_SetGimbal(const GimbalCmd& command)
{
    if (!Input_Armed) // 输入许可关闭时拒绝外部设置，不能覆盖失联安全目标。
        return;
    Gimbal_Command = command;
    Gimbal_Command_Dirty = true;
}

/**
 * @brief 缓存底盘目标，由 Update 的固定分频发布，不设置变化标志。
 * @note 仅输入许可成立时接受，普通调用最长等待剩余 10 周期分频。
 */
void RobotCmd_SetChassis(const ChassisCmd& command)
{
    if (!Input_Armed) // 输入许可关闭时拒绝外部设置，不能覆盖失联安全目标。
        return;
    Chassis_Command = command;
}

/**
 * @brief 缓存发射持续目标并标记待发布，仅在输入许可成立时接受。
 * @note 单发/三连发使用事件接口，不能用持续目标表示按钮边沿。
 */
void RobotCmd_SetShoot(const ShootCmd& command)
{
    if (!Input_Armed) // 输入许可关闭时拒绝外部设置，不能覆盖失联安全目标。
        return;
    Shoot_Command = command;
    Shoot_Command_Dirty = true;
}

/**
 * @brief 提交一次离散射击请求到容量 8 的 FIFO。
 * @return 输入许可关闭或队列已满返回 false；true 仅表示入队成功。
 * @note 不检查 ShootMode、摩擦轮达速或设备 ready，不确认物理发射完成。
 */
bool RobotCmd_PushShootEvent(const ShootEvent& event)
{
    return Input_Armed && MessageCenter::Shoot_Event_Queue.Push(event);
}

/**
 * @brief 读取最近 100 ms 内的云台反馈，失败时保持调用者对象不变。
 * @note 成功只表示消息新鲜；调用者仍需检查 online、enabled 与 ins_valid。
 */
bool RobotCmd_GetGimbalFeedback(GimbalFeedback& feedback)
{
    return MessageCenter::Gimbal_Feedback_Topic.ReadFresh(feedback, FEEDBACK_MAX_AGE_US);
}

/**
 * @brief 读取最近 100 ms 内的底盘反馈，失败时保持调用者对象不变。
 * @note 成功只表示消息新鲜，运行状态仍需检查 online 和 enabled。
 */
bool RobotCmd_GetChassisFeedback(ChassisFeedback& feedback)
{
    return MessageCenter::Chassis_Feedback_Topic.ReadFresh(feedback, FEEDBACK_MAX_AGE_US);
}

/**
 * @brief 读取最近 100 ms 内的发射反馈，失败时保持调用者对象不变。
 * @note enabled 是设备 ready 与总开关状态，不是达速或射击完成信号。
 */
bool RobotCmd_GetShootFeedback(ShootFeedback& feedback)
{
    return MessageCenter::Shoot_Feedback_Topic.ReadFresh(feedback, FEEDBACK_MAX_AGE_US);
}

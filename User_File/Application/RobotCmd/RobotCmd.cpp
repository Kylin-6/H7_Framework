/**
 * @file RobotCmd.cpp
 * @brief 机器人命令所有权与分发中心，参考 Meta-Embedded-NG 设计。
 * @details
 * RobotCmd 是各 Application 命令的唯一发布者，同时订阅各模块反馈。它只负责
 * 命令组织与模块间通信，不直接访问电机、CAN 或 IMU 设备。
 */

#include "RobotCmd.h"

#include "message_center.h"
#include "source_arbitration.h"

static Output<GimbalCmd> Gimbal_Command_Output;
static Output<ChassisCmd> Chassis_Command_Output;
static Output<ShootCmd> Shoot_Command_Output;
static Subscriber<GimbalFeedback> Gimbal_Feedback_Subscriber(
    MessageCenter::Gimbal_Feedback_Topic);
static Subscriber<ShootFeedback> Shoot_Feedback_Subscriber(
    MessageCenter::Shoot_Feedback_Topic);

static GimbalCmd Gimbal_Command;
static ChassisCmd Chassis_Command;
static ShootCmd Shoot_Command;
static GimbalFeedback Gimbal_Feedback;
static ShootFeedback Shoot_Feedback;

/* 云台和发射按变化发布；底盘命令按 10 ms 刷新以提供失联时效。 */
static bool Gimbal_Command_Dirty;
static bool Shoot_Command_Dirty;
static bool Chassis_Command_Dirty;
static bool Input_Armed;
static bool Gimbal_Feedback_Valid;
static bool Shoot_Feedback_Valid;
static uint8_t RobotCmd_Feedback_Divider;
static InputSource Last_Input_Source;
static void RobotCmd_SetInputArmed(bool armed);

static bool GimbalChanged(const GimbalCmd &next)
{
    return Gimbal_Command.mode != next.mode ||
           Gimbal_Command.yaw_angle_rad != next.yaw_angle_rad ||
           Gimbal_Command.pitch_angle_rad != next.pitch_angle_rad ||
           Gimbal_Command.yaw_speed_rad_s != next.yaw_speed_rad_s ||
           Gimbal_Command.pitch_speed_rad_s != next.pitch_speed_rad_s;
}

static bool ShootChanged(const ShootCmd &next)
{
    return Shoot_Command.shoot_mode != next.shoot_mode ||
           Shoot_Command.friction_mode != next.friction_mode ||
           Shoot_Command.loader_mode != next.loader_mode ||
           Shoot_Command.friction_speed_rad_s != next.friction_speed_rad_s ||
           Shoot_Command.loader_speed_rad_s != next.loader_speed_rad_s ||
           Shoot_Command.shoot_rate_hz != next.shoot_rate_hz;
}

bool RobotCmd_Init(Output<GimbalCmd> gimbal_output,
                   Output<ChassisCmd> chassis_output,
                   Output<ShootCmd> shoot_output)
{
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
    Gimbal_Feedback_Valid = false;
    Shoot_Feedback_Valid = false;
    RobotCmd_Feedback_Divider = 0U;
    Last_Input_Source = InputSource::Remote;
    return true;
}

void RobotCmd_Update(void)
{
    const InputDecision decision = SourceArbitration_Resolve(InputState_Read());
    const bool source_changed = decision.source != Last_Input_Source;
    RobotCmd_SetInputArmed(decision.armed);
    if (decision.armed)
    {
        if (source_changed)
        {
            Chassis_Command_Dirty = true;
            ShootEvent discarded{};
            while (MessageCenter::Shoot_Event_Queue.Pop(discarded)) {}
        }
        if (GimbalChanged(decision.gimbal))
        {
            RobotCmd_SetGimbal(decision.gimbal);
        }
        RobotCmd_SetChassis(decision.chassis);
        if (ShootChanged(decision.shoot))
        {
            RobotCmd_SetShoot(decision.shoot);
        }
    }
    Last_Input_Source = decision.source;

    /* 应用反馈以 100 Hz 拉取，首次未收到反馈时 Valid 保持 false。 */
    if (RobotCmd_Feedback_Divider == 0U)
    {
        GimbalFeedback gimbal_feedback;
        ShootFeedback shoot_feedback;

        if (Gimbal_Feedback_Subscriber.Read(gimbal_feedback))
        {
            Gimbal_Feedback = gimbal_feedback;
            Gimbal_Feedback_Valid = true;
        }
        if (Shoot_Feedback_Subscriber.Read(shoot_feedback))
        {
            Shoot_Feedback = shoot_feedback;
            Shoot_Feedback_Valid = true;
        }
    }
    const bool chassis_publish_due = RobotCmd_Feedback_Divider == 0U;
    RobotCmd_Feedback_Divider = (RobotCmd_Feedback_Divider + 1U) % 10U;

    /* 云台和发射按变化发布，底盘命令固定周期刷新。 */
    if (Gimbal_Command_Dirty)
    {
        Gimbal_Command_Output.Publish(Gimbal_Command);
        Gimbal_Command_Dirty = false;
    }
    // Continuous command refresh doubles as the receiver's freshness source.
    if (chassis_publish_due || Chassis_Command_Dirty)
    {
        Chassis_Command_Output.Publish(Chassis_Command);
        Chassis_Command_Dirty = false;
    }
    if (Shoot_Command_Dirty)
    {
        Shoot_Command_Output.Publish(Shoot_Command);
        Shoot_Command_Dirty = false;
    }
}

static void RobotCmd_SetInputArmed(bool armed)
{
    if (armed == Input_Armed)
    {
        return;
    }
    Input_Armed = armed;
    if (!armed)
    {
        Gimbal_Command = {};
        Chassis_Command = {};
        Shoot_Command = {};
        Gimbal_Command_Dirty = true;
        Chassis_Command_Dirty = true;
        Shoot_Command_Dirty = true;
        ShootEvent discarded{};
        while (MessageCenter::Shoot_Event_Queue.Pop(discarded)) {}
    }
}

void RobotCmd_SetGimbal(const GimbalCmd &command)
{
    if (!Input_Armed) return;
    Gimbal_Command = command;
    Gimbal_Command_Dirty = true;
}

void RobotCmd_SetChassis(const ChassisCmd &command)
{
    if (!Input_Armed) return;
    Chassis_Command = command;
}

void RobotCmd_SetShoot(const ShootCmd &command)
{
    if (!Input_Armed) return;
    Shoot_Command = command;
    Shoot_Command_Dirty = true;
}

bool RobotCmd_PushShootEvent(const ShootEvent &event)
{
    return Input_Armed && MessageCenter::Shoot_Event_Queue.Push(event);
}

bool RobotCmd_GetGimbalFeedback(GimbalFeedback &feedback)
{
    if (!Gimbal_Feedback_Valid)
    {
        return false;
    }
    feedback = Gimbal_Feedback;
    return true;
}

bool RobotCmd_GetChassisFeedback(ChassisFeedback &feedback)
{
    return MessageCenter::Chassis_Feedback_Topic.ReadFresh(feedback, 100000U);
}

bool RobotCmd_GetShootFeedback(ShootFeedback &feedback)
{
    if (!Shoot_Feedback_Valid)
    {
        return false;
    }
    feedback = Shoot_Feedback;
    return true;
}

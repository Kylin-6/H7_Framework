/**
 * @file RobotCmd.cpp
 * @brief 机器人命令所有权与分发中心，参考 Meta-Embedded-NG 设计。
 * @details
 * RobotCmd 是各 Application 命令的唯一发布者，同时订阅各模块反馈。它只负责
 * 命令组织与模块间通信，不直接访问电机、CAN 或 IMU 设备。
 */

#include "RobotCmd.h"

#include "message_center.h"

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
static bool Gimbal_Feedback_Valid;
static bool Shoot_Feedback_Valid;
static uint8_t RobotCmd_Feedback_Divider;

void RobotCmd_Init(Output<GimbalCmd> gimbal_output,
                   Output<ChassisCmd> chassis_output,
                   Output<ShootCmd> shoot_output)
{
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
    Gimbal_Feedback_Valid = false;
    Shoot_Feedback_Valid = false;
    RobotCmd_Feedback_Divider = 0U;
}

void RobotCmd_Update(void)
{
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
    if (chassis_publish_due)
    {
        Chassis_Command_Output.Publish(Chassis_Command);
    }
    if (Shoot_Command_Dirty)
    {
        Shoot_Command_Output.Publish(Shoot_Command);
        Shoot_Command_Dirty = false;
    }
}

void RobotCmd_SetGimbal(const GimbalCmd &command)
{
    Gimbal_Command = command;
    Gimbal_Command_Dirty = true;
}

void RobotCmd_SetChassis(const ChassisCmd &command)
{
    Chassis_Command = command;
}

void RobotCmd_SetShoot(const ShootCmd &command)
{
    Shoot_Command = command;
    Shoot_Command_Dirty = true;
}

bool RobotCmd_PushShootEvent(const ShootEvent &event)
{
    return MessageCenter::Shoot_Event_Queue.Push(event);
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

/**
 * @file RobotCmd.cpp
 * @brief Command ownership and distribution adapted from Meta-Embedded-NG.
 */

#include "RobotCmd.h"

#include "application_topics.h"
#include "dynamic_message_center.h"

static DynamicPublisher_t *Gimbal_Command_Publisher;
static DynamicPublisher_t *Chassis_Command_Publisher;
static DynamicPublisher_t *Shoot_Command_Publisher;
static DynamicSubscriber_t *Gimbal_Feedback_Subscriber;
static DynamicSubscriber_t *Chassis_Feedback_Subscriber;
static DynamicSubscriber_t *Shoot_Feedback_Subscriber;

static GimbalCmd Gimbal_Command;
static ChassisCmd Chassis_Command;
static ShootCmd Shoot_Command;
static GimbalFeedback Gimbal_Feedback;
static ChassisFeedback Chassis_Feedback;
static ShootFeedback Shoot_Feedback;

static bool Gimbal_Command_Dirty;
static bool Chassis_Command_Dirty;
static bool Shoot_Command_Dirty;
static bool Gimbal_Feedback_Valid;
static bool Chassis_Feedback_Valid;
static bool Shoot_Feedback_Valid;
static uint8_t RobotCmd_Feedback_Divider;

bool RobotCmd_RegisterTopics(void)
{
    Gimbal_Command_Publisher = DynamicPublisher_Register(
        APPLICATION_TOPIC_GIMBAL_CMD, sizeof(GimbalCmd));
    Chassis_Command_Publisher = DynamicPublisher_Register(
        APPLICATION_TOPIC_CHASSIS_CMD, sizeof(ChassisCmd));
    Shoot_Command_Publisher = DynamicPublisher_Register(
        APPLICATION_TOPIC_SHOOT_CMD, sizeof(ShootCmd));

    Gimbal_Feedback_Subscriber = DynamicSubscriber_Register(
        APPLICATION_TOPIC_GIMBAL_FEEDBACK, sizeof(GimbalFeedback));
    Chassis_Feedback_Subscriber = DynamicSubscriber_Register(
        APPLICATION_TOPIC_CHASSIS_FEEDBACK, sizeof(ChassisFeedback));
    Shoot_Feedback_Subscriber = DynamicSubscriber_Register(
        APPLICATION_TOPIC_SHOOT_FEEDBACK, sizeof(ShootFeedback));

    return Gimbal_Command_Publisher != nullptr &&
           Chassis_Command_Publisher != nullptr &&
           Shoot_Command_Publisher != nullptr &&
           Gimbal_Feedback_Subscriber != nullptr &&
           Chassis_Feedback_Subscriber != nullptr &&
           Shoot_Feedback_Subscriber != nullptr;
}

void RobotCmd_Init(void)
{
    Gimbal_Command = {};
    Chassis_Command = {};
    Shoot_Command = {};
    // Safe RM startup defaults: gimbal holds the pose captured by Gimbal_Init,
    // while chassis and shooter remain zero-force/off.
    Gimbal_Command.mode = GimbalMode::LOCK;
    Gimbal_Command_Dirty = true;
    Chassis_Command_Dirty = true;
    Shoot_Command_Dirty = true;
    Gimbal_Feedback_Valid = false;
    Chassis_Feedback_Valid = false;
    Shoot_Feedback_Valid = false;
    RobotCmd_Feedback_Divider = 0U;
}

void RobotCmd_Update(void)
{
    if (RobotCmd_Feedback_Divider == 0U)
    {
        GimbalFeedback gimbal_feedback;
        ChassisFeedback chassis_feedback;
        ShootFeedback shoot_feedback;

        if (DynamicSubscriber_Read(Gimbal_Feedback_Subscriber, &gimbal_feedback))
        {
            Gimbal_Feedback = gimbal_feedback;
            Gimbal_Feedback_Valid = true;
        }
        if (DynamicSubscriber_Read(Chassis_Feedback_Subscriber, &chassis_feedback))
        {
            Chassis_Feedback = chassis_feedback;
            Chassis_Feedback_Valid = true;
        }
        if (DynamicSubscriber_Read(Shoot_Feedback_Subscriber, &shoot_feedback))
        {
            Shoot_Feedback = shoot_feedback;
            Shoot_Feedback_Valid = true;
        }
    }
    RobotCmd_Feedback_Divider = (RobotCmd_Feedback_Divider + 1U) % 10U;

    if (Gimbal_Command_Dirty)
    {
        DynamicPublisher_Publish(Gimbal_Command_Publisher, &Gimbal_Command);
        Gimbal_Command_Dirty = false;
    }
    if (Chassis_Command_Dirty)
    {
        DynamicPublisher_Publish(Chassis_Command_Publisher, &Chassis_Command);
        Chassis_Command_Dirty = false;
    }
    if (Shoot_Command_Dirty)
    {
        DynamicPublisher_Publish(Shoot_Command_Publisher, &Shoot_Command);
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
    Chassis_Command_Dirty = true;
}

void RobotCmd_SetShoot(const ShootCmd &command)
{
    Shoot_Command = command;
    Shoot_Command_Dirty = true;
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
    if (!Chassis_Feedback_Valid)
    {
        return false;
    }
    feedback = Chassis_Feedback;
    return true;
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

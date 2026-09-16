/**
 * @file Shoot.cpp
 * @brief Basic friction-wheel and loader application adapted from Meta-Embedded-NG.
 *
 * Heat and jam thresholds are intentionally not copied: they depend on the
 * actual mechanism and referee data that are not present in this project.
 */

#include "Shoot.h"

#include "application_topics.h"
#include "dynamic_message_center.h"
#include "message_types.h"

#if SHOOT
#include "dji_motor.h"
#include "fdcan.h"
#include <cmath>
#endif

static DynamicSubscriber_t *Shoot_Command_Subscriber;
static DynamicPublisher_t *Shoot_Feedback_Publisher;
static ShootCmd Shoot_Command;
static ShootFeedback Shoot_Feedback;
static uint8_t Shoot_Feedback_Divider;

#if SHOOT
static constexpr float SHOOT_DEFAULT_FRICTION_SPEED_DEG_S = 40000.0f;
static constexpr float SHOOT_DEFAULT_RATE_HZ = 10.0f;
static constexpr float SHOOT_ONE_BULLET_ANGLE_DEG = 36.0f;
static constexpr float SHOOT_REVERSE_SPEED_DEG_S = -360.0f;

static Class_DJIMotor Shoot_Friction_Left;
static Class_DJIMotor Shoot_Friction_Right;
static Class_DJIMotor Shoot_Loader;
static Class_DJIMotor_Group Shoot_Friction_Group;
static Class_DJIMotor_Group Shoot_Loader_Group;
static bool Shoot_Initialized;
static bool Shoot_Output_Enabled;
static LoaderMode Shoot_Last_Loader_Mode;
static float Shoot_Loader_Angle_Target;

static PID_InitTypeDef Shoot_MakePID(float kp, float ki, float kd,
                                    float integral_limit, float output_limit)
{
    PID_InitTypeDef pid{};
    pid.K_P = kp;
    pid.K_I = ki;
    pid.K_D = kd;
    pid.I_Out_Max = integral_limit;
    pid.Out_Max = output_limit;
    pid.D_T = 0.001f;
    return pid;
}

static void Shoot_SetEnabled(bool enabled)
{
    if (enabled == Shoot_Output_Enabled)
    {
        return;
    }
    Shoot_Output_Enabled = enabled;
    if (enabled)
    {
        Shoot_Friction_Group.Enable();
        Shoot_Loader_Group.Enable();
    }
    else
    {
        Shoot_Friction_Group.Disable();
        Shoot_Loader_Group.Disable();
    }
}

static void Shoot_ApplyCommand(void)
{
    const bool enabled = Shoot_Command.shoot_mode == ShootMode::ON;
    Shoot_SetEnabled(enabled);
    if (!enabled)
    {
        Shoot_Last_Loader_Mode = LoaderMode::STOP;
        return;
    }

    float friction_reference = 0.0f;
    if (Shoot_Command.friction_mode == FrictionMode::ON)
    {
        friction_reference = Shoot_Command.friction_speed_deg_s > 0.0f
            ? Shoot_Command.friction_speed_deg_s
            : SHOOT_DEFAULT_FRICTION_SPEED_DEG_S;
    }
    Shoot_Friction_Group.Control(friction_reference, friction_reference);

    float loader_reference = 0.0f;
    switch (Shoot_Command.loader_mode)
    {
    case LoaderMode::SINGLE:
    case LoaderMode::TRIPLE:
        if (Shoot_Command.loader_mode != Shoot_Last_Loader_Mode)
        {
            const float bullet_count =
                Shoot_Command.loader_mode == LoaderMode::SINGLE ? 1.0f : 3.0f;
            Shoot_Loader_Angle_Target = Shoot_Loader.feedback.output_total_angle +
                bullet_count * SHOOT_ONE_BULLET_ANGLE_DEG;
        }
        Shoot_Loader.Set_Outer_Loop(DJI_MOTOR_ANGLE_LOOP);
        loader_reference = Shoot_Loader_Angle_Target;
        break;

    case LoaderMode::BURST:
    {
        Shoot_Loader.Set_Outer_Loop(DJI_MOTOR_SPEED_LOOP);
        const float rate = Shoot_Command.shoot_rate_hz > 0.0f
            ? Shoot_Command.shoot_rate_hz : SHOOT_DEFAULT_RATE_HZ;
        loader_reference = Shoot_Command.loader_speed_deg_s != 0.0f
            ? Shoot_Command.loader_speed_deg_s
            : rate * SHOOT_ONE_BULLET_ANGLE_DEG;
        break;
    }

    case LoaderMode::REVERSE:
        Shoot_Loader.Set_Outer_Loop(DJI_MOTOR_SPEED_LOOP);
        loader_reference = Shoot_Command.loader_speed_deg_s != 0.0f
            ? -std::fabs(Shoot_Command.loader_speed_deg_s)
            : SHOOT_REVERSE_SPEED_DEG_S;
        break;

    case LoaderMode::STOP:
    default:
        Shoot_Loader.Set_Outer_Loop(DJI_MOTOR_SPEED_LOOP);
        loader_reference = 0.0f;
        break;
    }

    Shoot_Loader_Group.Control(loader_reference);
    Shoot_Last_Loader_Mode = Shoot_Command.loader_mode;
}

static void Shoot_UpdateFeedback(void)
{
    Shoot_Feedback.friction_left_speed_deg_s =
        Shoot_Friction_Left.feedback.output_speed;
    Shoot_Feedback.friction_right_speed_deg_s =
        Shoot_Friction_Right.feedback.output_speed;
    Shoot_Feedback.loader_angle_deg = Shoot_Loader.feedback.output_total_angle;
    Shoot_Feedback.loader_speed_deg_s = Shoot_Loader.feedback.output_speed;
    Shoot_Feedback.enabled = Shoot_Output_Enabled;
    Shoot_Feedback.online = Shoot_Friction_Left.online &&
                            Shoot_Friction_Right.online &&
                            Shoot_Loader.online;
}
#endif

bool Shoot_RegisterTopics(void)
{
    Shoot_Command_Subscriber = DynamicSubscriber_Register(
        APPLICATION_TOPIC_SHOOT_CMD, sizeof(ShootCmd));
    Shoot_Feedback_Publisher = DynamicPublisher_Register(
        APPLICATION_TOPIC_SHOOT_FEEDBACK, sizeof(ShootFeedback));
    return Shoot_Command_Subscriber != nullptr &&
           Shoot_Feedback_Publisher != nullptr;
}

bool Shoot_Init(void)
{
    Shoot_Command = {};
    Shoot_Feedback = {};
    Shoot_Feedback_Divider = 0U;

#if SHOOT
    Struct_DJIMotor_Init_Config friction_config{};
    friction_config.hfdcan = &hfdcan3;
    friction_config.motor_type = Enum_DJIMotor_Type::M3508;
    friction_config.close_loop = DJI_MOTOR_SPEED_LOOP;
    friction_config.outer_loop = DJI_MOTOR_SPEED_LOOP;
    friction_config.speed_pid = Shoot_MakePID(7.5f, 5.0f, 0.0f, 16000.0f, 16000.0f);

    friction_config.can_id = 3U;
    const bool left_initialized = Shoot_Friction_Left.Init(friction_config);
    friction_config.can_id = 2U;
    friction_config.reverse = true;
    const bool right_initialized = Shoot_Friction_Right.Init(friction_config);

    Struct_DJIMotor_Init_Config loader_config{};
    loader_config.hfdcan = &hfdcan3;
    loader_config.can_id = 8U;
    loader_config.motor_type = Enum_DJIMotor_Type::M3508;
    loader_config.close_loop = DJI_MOTOR_CURRENT_LOOP |
                               DJI_MOTOR_SPEED_LOOP |
                               DJI_MOTOR_ANGLE_LOOP;
    loader_config.outer_loop = DJI_MOTOR_SPEED_LOOP;
    loader_config.current_pid = Shoot_MakePID(1.0f, 50.0f, 0.0f, 12000.0f, 12000.0f);
    loader_config.speed_pid = Shoot_MakePID(7.5f, 20.0f, 0.0f, 12000.0f, 12000.0f);
    loader_config.angle_pid = Shoot_MakePID(10.0f, 0.0f, 0.0f, 0.0f, 360.0f);
    const bool loader_initialized = Shoot_Loader.Init(loader_config);

    Shoot_Initialized = left_initialized && right_initialized && loader_initialized &&
        Shoot_Friction_Group.Init(&Shoot_Friction_Left, &Shoot_Friction_Right) &&
        Shoot_Loader_Group.Init(&Shoot_Loader);
    Shoot_Output_Enabled = true;
    if (Shoot_Initialized)
    {
        Shoot_SetEnabled(false);
    }
    Shoot_Last_Loader_Mode = LoaderMode::STOP;
    Shoot_Loader_Angle_Target = 0.0f;
    return Shoot_Initialized;
#else
    return true;
#endif
}

void Shoot_Update(void)
{
    ShootCmd command;
    if (DynamicSubscriber_Read(Shoot_Command_Subscriber, &command))
    {
        Shoot_Command = command;
    }

#if SHOOT
    if (Shoot_Initialized)
    {
        Shoot_ApplyCommand();
        Shoot_UpdateFeedback();
    }
#endif

    Shoot_Feedback_Divider++;
    if (Shoot_Feedback_Divider >= 10U)
    {
        Shoot_Feedback_Divider = 0U;
        DynamicPublisher_Publish(Shoot_Feedback_Publisher, &Shoot_Feedback);
    }
}

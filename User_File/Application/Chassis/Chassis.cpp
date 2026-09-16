/**
 * @file Chassis.cpp
 * @brief Four-module AGV chassis adapted from Meta-Embedded-NG.
 *
 * The kinematics and steering shortest-path rule come from the MIT-licensed
 * Meta-Embedded-NG application/chassis implementation. Motor access is adapted
 * to this project's Class_DJIMotor API. Mechanical constants remain calibration
 * values and the module is disabled by default.
 */

#include "Chassis.h"

#include "application_topics.h"
#include "dynamic_message_center.h"
#include "message_types.h"

#if CHASSIS
#include "dji_motor.h"
#include "fdcan.h"
#include <cmath>
#endif

static DynamicSubscriber_t *Chassis_Command_Subscriber;
static DynamicPublisher_t *Chassis_Feedback_Publisher;
static ChassisCmd Chassis_Command;
static ChassisFeedback Chassis_Feedback;
static uint8_t Chassis_Feedback_Divider;

#if CHASSIS
static constexpr float CHASSIS_HALF_LENGTH_M = 0.163f;
static constexpr float CHASSIS_HALF_WIDTH_M = 0.163f;
static constexpr float CHASSIS_WHEEL_RADIUS_M = 0.058f;
static constexpr float CHASSIS_WHEEL_PERIMETER_M =
    2.0f * 3.14159265358979323846f * CHASSIS_WHEEL_RADIUS_M;
static constexpr float CHASSIS_FEEDBACK_ALPHA = 0.032258f;
static constexpr float CHASSIS_STOP_SPEED_M_S = 0.001f;
static constexpr float Chassis_Steer_Offset_Deg[4] = {
    102.5f, 12.5f, 137.5f, 145.0f};

static Class_DJIMotor Chassis_Wheel_Motor[4];
static Class_DJIMotor Chassis_Steer_Motor[4];
static Class_DJIMotor_Group Chassis_Wheel_Group;
static Class_DJIMotor_Group Chassis_Steer_Group;
static bool Chassis_Initialized;
static bool Chassis_Output_Enabled;
static int8_t Chassis_Wheel_Direction[4] = {1, 1, 1, 1};

static PID_InitTypeDef Chassis_MakePID(float kp, float ki, float kd,
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

static float Chassis_NormalizeAngle(float angle_deg)
{
    while (angle_deg > 180.0f)
    {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

static void Chassis_SetEnabled(bool enabled)
{
    if (enabled == Chassis_Output_Enabled)
    {
        return;
    }
    Chassis_Output_Enabled = enabled;
    if (enabled)
    {
        Chassis_Wheel_Group.Enable();
        Chassis_Steer_Group.Enable();
    }
    else
    {
        Chassis_Wheel_Group.Disable();
        Chassis_Steer_Group.Disable();
    }
}

static void Chassis_CalculateTargets(float wheel_target[4], float steer_target[4])
{
    const float vx = Chassis_Command.velocity_x_m_s;
    const float vy = Chassis_Command.velocity_y_m_s;
    const float wz = Chassis_Command.angular_velocity_rad_s;
    const float wheel_vx[4] = {
        vx + wz * CHASSIS_HALF_WIDTH_M,
        vx + wz * CHASSIS_HALF_WIDTH_M,
        vx - wz * CHASSIS_HALF_WIDTH_M,
        vx - wz * CHASSIS_HALF_WIDTH_M,
    };
    const float wheel_vy[4] = {
        vy + wz * CHASSIS_HALF_LENGTH_M,
        vy - wz * CHASSIS_HALF_LENGTH_M,
        vy - wz * CHASSIS_HALF_LENGTH_M,
        vy + wz * CHASSIS_HALF_LENGTH_M,
    };

    for (uint8_t index = 0; index < 4; ++index)
    {
        const float velocity = std::sqrt(wheel_vx[index] * wheel_vx[index] +
                                         wheel_vy[index] * wheel_vy[index]);
        const float current_angle =
            Chassis_Steer_Motor[index].feedback.output_total_angle;
        if (velocity < CHASSIS_STOP_SPEED_M_S)
        {
            wheel_target[index] = 0.0f;
            steer_target[index] = current_angle;
            continue;
        }

        float target_angle = std::atan2(wheel_vy[index], wheel_vx[index]) *
                             (180.0f / 3.14159265358979323846f) +
                             Chassis_Steer_Offset_Deg[index];
        float difference = Chassis_NormalizeAngle(target_angle - current_angle);
        if (difference > 90.0f)
        {
            difference -= 180.0f;
            Chassis_Wheel_Direction[index] = -1;
        }
        else if (difference < -90.0f)
        {
            difference += 180.0f;
            Chassis_Wheel_Direction[index] = -1;
        }
        else
        {
            Chassis_Wheel_Direction[index] = 1;
        }

        steer_target[index] = current_angle + difference;
        wheel_target[index] = velocity * (360.0f / CHASSIS_WHEEL_PERIMETER_M) *
                              Chassis_Wheel_Direction[index];
    }
}

static void Chassis_UpdateFeedback(void)
{
    float wheel_vx[4];
    float wheel_vy[4];
    bool online = true;
    for (uint8_t index = 0; index < 4; ++index)
    {
        const float heading =
            (Chassis_Steer_Motor[index].feedback.output_total_angle -
             Chassis_Steer_Offset_Deg[index]) *
            (3.14159265358979323846f / 180.0f);
        const float linear_speed =
            Chassis_Wheel_Motor[index].feedback.output_speed *
            (CHASSIS_WHEEL_PERIMETER_M / 360.0f);
        wheel_vx[index] = linear_speed * std::cos(heading);
        wheel_vy[index] = linear_speed * std::sin(heading);
        online = online && Chassis_Wheel_Motor[index].online &&
                 Chassis_Steer_Motor[index].online;
    }

    const float vx = (wheel_vx[0] + wheel_vx[1] + wheel_vx[2] + wheel_vx[3]) * 0.25f;
    const float vy = (wheel_vy[0] + wheel_vy[1] + wheel_vy[2] + wheel_vy[3]) * 0.25f;
    const float wz_x = ((wheel_vx[0] - wheel_vx[2]) +
                        (wheel_vx[1] - wheel_vx[3])) /
                       (4.0f * CHASSIS_HALF_WIDTH_M);
    const float wz_y = ((wheel_vy[0] - wheel_vy[1]) +
                        (wheel_vy[3] - wheel_vy[2])) /
                       (4.0f * CHASSIS_HALF_LENGTH_M);

    Chassis_Feedback.velocity_x_m_s += CHASSIS_FEEDBACK_ALPHA *
        (vx - Chassis_Feedback.velocity_x_m_s);
    Chassis_Feedback.velocity_y_m_s += CHASSIS_FEEDBACK_ALPHA *
        (vy - Chassis_Feedback.velocity_y_m_s);
    Chassis_Feedback.angular_velocity_rad_s += CHASSIS_FEEDBACK_ALPHA *
        (0.5f * (wz_x + wz_y) - Chassis_Feedback.angular_velocity_rad_s);
    Chassis_Feedback.enabled = Chassis_Output_Enabled;
    Chassis_Feedback.online = online;
}
#endif

bool Chassis_RegisterTopics(void)
{
    Chassis_Command_Subscriber = DynamicSubscriber_Register(
        APPLICATION_TOPIC_CHASSIS_CMD, sizeof(ChassisCmd));
    Chassis_Feedback_Publisher = DynamicPublisher_Register(
        APPLICATION_TOPIC_CHASSIS_FEEDBACK, sizeof(ChassisFeedback));
    return Chassis_Command_Subscriber != nullptr &&
           Chassis_Feedback_Publisher != nullptr;
}

bool Chassis_Init(void)
{
    Chassis_Command = {};
    Chassis_Feedback = {};
    Chassis_Feedback_Divider = 0U;

#if CHASSIS
    Struct_DJIMotor_Init_Config wheel_config{};
    wheel_config.hfdcan = &hfdcan1;
    wheel_config.motor_type = Enum_DJIMotor_Type::M3508;
    wheel_config.close_loop = DJI_MOTOR_SPEED_LOOP;
    wheel_config.outer_loop = DJI_MOTOR_SPEED_LOOP;
    wheel_config.speed_pid = Chassis_MakePID(4.5f, 0.05f, 0.0f, 3000.0f, 16000.0f);

    Struct_DJIMotor_Init_Config steer_config{};
    steer_config.hfdcan = &hfdcan2;
    steer_config.motor_type = Enum_DJIMotor_Type::M3508;
    steer_config.close_loop = DJI_MOTOR_ANGLE_LOOP | DJI_MOTOR_SPEED_LOOP;
    steer_config.outer_loop = DJI_MOTOR_ANGLE_LOOP;
    steer_config.angle_pid = Chassis_MakePID(30.0f, 0.2f, 0.0f, 200.0f, 1000.0f);
    steer_config.speed_pid = Chassis_MakePID(4.0f, 4.0f, 0.0f, 3000.0f, 15000.0f);

    bool initialized = true;
    for (uint8_t index = 0; index < 4; ++index)
    {
        wheel_config.can_id = index + 1U;
        steer_config.can_id = index + 1U;
        initialized = Chassis_Wheel_Motor[index].Init(wheel_config) && initialized;
        initialized = Chassis_Steer_Motor[index].Init(steer_config) && initialized;
    }
    initialized = initialized && Chassis_Wheel_Group.Init(
        &Chassis_Wheel_Motor[0], &Chassis_Wheel_Motor[1],
        &Chassis_Wheel_Motor[2], &Chassis_Wheel_Motor[3]);
    initialized = initialized && Chassis_Steer_Group.Init(
        &Chassis_Steer_Motor[0], &Chassis_Steer_Motor[1],
        &Chassis_Steer_Motor[2], &Chassis_Steer_Motor[3]);
    Chassis_Initialized = initialized;
    Chassis_Output_Enabled = true;
    if (initialized)
    {
        Chassis_SetEnabled(false);
    }
    return initialized;
#else
    return true;
#endif
}

void Chassis_Update(void)
{
    ChassisCmd command;
    if (DynamicSubscriber_Read(Chassis_Command_Subscriber, &command))
    {
        Chassis_Command = command;
    }

#if CHASSIS
    if (Chassis_Initialized)
    {
        const bool enabled = Chassis_Command.mode != ChassisMode::ZERO_FORCE;
        Chassis_SetEnabled(enabled);
        if (enabled)
        {
            float wheel_target[4];
            float steer_target[4];
            Chassis_CalculateTargets(wheel_target, steer_target);
            Chassis_Wheel_Group.Control(wheel_target[0], wheel_target[1],
                                        wheel_target[2], wheel_target[3]);
            Chassis_Steer_Group.Control(steer_target[0], steer_target[1],
                                        steer_target[2], steer_target[3]);
        }
        Chassis_UpdateFeedback();
    }
#endif

    Chassis_Feedback_Divider++;
    if (Chassis_Feedback_Divider >= 10U)
    {
        Chassis_Feedback_Divider = 0U;
        DynamicPublisher_Publish(Chassis_Feedback_Publisher, &Chassis_Feedback);
    }
}

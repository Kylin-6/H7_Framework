/**
 * @file Chassis.cpp
 * @brief 基于四个舵轮模块的 AGV 底盘应用，参考 Meta-Embedded-NG 移植。
 *
 * 运动学与舵向最短路径规则来自 MIT 许可证下的 Meta-Embedded-NG
 * application/chassis，实现已适配本工程 Class_DJIMotor 接口。机械参数仍是
 * 待实车标定值；双板构建将此模块放在底盘板。
 * @todo vx/vy/wz 的实车物理正方向须按电机安装和坐标系标定，不能仅由数组顺序推断。
 * @todo 舵向依赖 output_total_angle；增量编码器上电不提供绝对输出轴零位，
 *       需绝对编码器、寻零流程或已知上电姿态。
 */

#include "Chassis.h"
#include "../physical_units.h"

#include "message_center.h"
#include "board_config.h"

static constexpr uint64_t CHASSIS_COMMAND_MAX_AGE_US = 100000U;

#if CHASSIS
#include "dji_motor.h"
#include "fdcan.h"
#include <cmath>
#endif

#if CHASSIS
static constexpr float CHASSIS_HALF_LENGTH_M = 0.163f;
static constexpr float CHASSIS_HALF_WIDTH_M = 0.163f;
static constexpr float CHASSIS_WHEEL_RADIUS_M = 0.058f;
static constexpr float CHASSIS_FEEDBACK_ALPHA = 0.032258f;
static constexpr float CHASSIS_STOP_SPEED_M_S = 0.001f;
static constexpr float Chassis_Steer_Offset_Rad[4] = {
    DegToRad(102.5f), DegToRad(12.5f),
    DegToRad(137.5f), DegToRad(145.0f)};

#endif

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
    bool output_enabled = false;
    int8_t wheel_direction[4] = {1, 1, 1, 1};
#endif
};

ChassisContext ctx;
}

#if CHASSIS

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

static void Chassis_SetEnabled(bool enabled)
{
    if (enabled == ctx.output_enabled)
    {
        return;
    }
    ctx.output_enabled = enabled;
    if (enabled)
    {
        ctx.wheel_group.Enable();
        ctx.steer_group.Enable();
    }
    else
    {
        ctx.wheel_group.Disable();
        ctx.steer_group.Disable();
    }
}

static void Chassis_CalculateTargets(float wheel_target_rad_s[4],
                                     float steer_target_rad[4])
{
    /* 四轮位置的旋转项为 ±wz·半宽/半长；符号按下方轮索引数组固定。
       物理前/左和正转方向必须由实车接线及坐标标定确认。 */
    const float vx_m_s = ctx.command.velocity_x_m_s;
    const float vy_m_s = ctx.command.velocity_y_m_s;
    const float wz_rad_s = ctx.command.angular_velocity_rad_s;
    const float wheel_vx[4] = {
        vx_m_s + wz_rad_s * CHASSIS_HALF_WIDTH_M,
        vx_m_s + wz_rad_s * CHASSIS_HALF_WIDTH_M,
        vx_m_s - wz_rad_s * CHASSIS_HALF_WIDTH_M,
        vx_m_s - wz_rad_s * CHASSIS_HALF_WIDTH_M,
    };
    const float wheel_vy[4] = {
        vy_m_s + wz_rad_s * CHASSIS_HALF_LENGTH_M,
        vy_m_s - wz_rad_s * CHASSIS_HALF_LENGTH_M,
        vy_m_s - wz_rad_s * CHASSIS_HALF_LENGTH_M,
        vy_m_s + wz_rad_s * CHASSIS_HALF_LENGTH_M,
    };

    for (uint8_t index = 0; index < 4; ++index)
    {
        const float velocity_m_s = std::sqrt(wheel_vx[index] * wheel_vx[index] +
                                            wheel_vy[index] * wheel_vy[index]);
        const float current_angle_rad =
            ctx.steer_snapshot[index].output_total_angle;
        if (velocity_m_s < CHASSIS_STOP_SPEED_M_S)
        {
            wheel_target_rad_s[index] = 0.0f;
            steer_target_rad[index] = current_angle_rad;
            continue;
        }

        const float target_angle_rad = std::atan2(wheel_vy[index], wheel_vx[index]) +
                                       Chassis_Steer_Offset_Rad[index];
        float difference_rad = std::remainder(target_angle_rad - current_angle_rad,
                                              2.0f * kPiRad);
        /* remainder 将误差压到 [-π, π]；超过 ±π/2 时舵角少转 π、轮速取反。 */
        if (difference_rad > kPiRad / 2.0f)
        {
            difference_rad -= kPiRad;
            ctx.wheel_direction[index] = -1;
        }
        else if (difference_rad < -kPiRad / 2.0f)
        {
            difference_rad += kPiRad;
            ctx.wheel_direction[index] = -1;
        }
        else
        {
            ctx.wheel_direction[index] = 1;
        }

        steer_target_rad[index] = current_angle_rad + difference_rad;
        wheel_target_rad_s[index] = (velocity_m_s / CHASSIS_WHEEL_RADIUS_M) *
                                    ctx.wheel_direction[index];
    }
}

static void Chassis_UpdateFeedback(void)
{
    float wheel_vx[4];
    float wheel_vy[4];
    bool online = true;
    for (uint8_t index = 0; index < 4; ++index)
    {
        const float heading_rad =
            ctx.steer_snapshot[index].output_total_angle -
            Chassis_Steer_Offset_Rad[index];
        const float linear_speed_m_s =
            ctx.wheel_snapshot[index].output_speed *
            CHASSIS_WHEEL_RADIUS_M;
        wheel_vx[index] = linear_speed_m_s * std::cos(heading_rad);
        wheel_vy[index] = linear_speed_m_s * std::sin(heading_rad);
        online = online && ctx.wheel_snapshot[index].online &&
                 ctx.steer_snapshot[index].online;
    }

    const float vx = (wheel_vx[0] + wheel_vx[1] + wheel_vx[2] + wheel_vx[3]) * 0.25f;
    const float vy = (wheel_vy[0] + wheel_vy[1] + wheel_vy[2] + wheel_vy[3]) * 0.25f;
    const float wz_x = ((wheel_vx[0] - wheel_vx[2]) +
                        (wheel_vx[1] - wheel_vx[3])) /
                       (4.0f * CHASSIS_HALF_WIDTH_M);
    const float wz_y = ((wheel_vy[0] - wheel_vy[1]) +
                        (wheel_vy[3] - wheel_vy[2])) /
                       (4.0f * CHASSIS_HALF_LENGTH_M);

    ctx.feedback.velocity_x_m_s += CHASSIS_FEEDBACK_ALPHA *
        (vx - ctx.feedback.velocity_x_m_s);
    ctx.feedback.velocity_y_m_s += CHASSIS_FEEDBACK_ALPHA *
        (vy - ctx.feedback.velocity_y_m_s);
    ctx.feedback.angular_velocity_rad_s += CHASSIS_FEEDBACK_ALPHA *
        (0.5f * (wz_x + wz_y) - ctx.feedback.angular_velocity_rad_s);
    ctx.feedback.enabled = ctx.output_enabled;
    ctx.feedback.online = online;
}
#endif

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
    wheel_config.speed_pid = Chassis_MakePID(4.5f, 0.05f, 0.0f, 3000.0f, 16000.0f);

    Struct_DJIMotor_Init_Config steer_config{};
    steer_config.hfdcan = BoardConfig_Get().chassis_steer_bus;
    steer_config.motor_type = Enum_DJIMotor_Type::M3508;
    steer_config.close_loop = DJI_MOTOR_ANGLE_LOOP | DJI_MOTOR_SPEED_LOOP;
    steer_config.outer_loop = DJI_MOTOR_ANGLE_LOOP;
    // 角度环输出为 rad/s：原 200/1000 deg/s 限幅作物理等效转换。
    // Kp/Ki 和舵轮速度环增益没有可信实车来源，启用前均需重新整定。
    steer_config.angle_pid = Chassis_MakePID(30.0f, 0.2f, 0.0f,
                                             DegToRad(200.0f), DegToRad(1000.0f));
    steer_config.speed_pid = Chassis_MakePID(4.0f, 4.0f, 0.0f, 3000.0f, 15000.0f);

    bool initialized = true;
    for (uint8_t index = 0; index < 4; ++index)
    {
        wheel_config.can_id = index + 1U;
        steer_config.can_id = index + 1U;
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
    ctx.output_enabled = true;
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
    /* A held target may only drive motors while its local Topic is fresh. */
    ChassisCmd command{};
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
    if (ctx.initialized)
    {
        for (uint8_t index = 0U; index < 4U; ++index)
        {
            ctx.wheel_snapshot[index] = ctx.wheel_motor[index].GetMotionSnapshot();
            ctx.steer_snapshot[index] = ctx.steer_motor[index].GetMotionSnapshot();
        }
        const bool enabled = ctx.command.mode != ChassisMode::ZERO_FORCE;
        Chassis_SetEnabled(enabled);
        if (enabled)
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
    if (ctx.feedback_divider >= 10U)
    {
        ctx.feedback_divider = 0U;
        ctx.feedback_publisher.Publish(ctx.feedback);
    }
}

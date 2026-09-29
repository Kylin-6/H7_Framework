/**
 * @file Shoot.cpp
 * @brief 摩擦轮与拨弹盘应用，参考 Meta-Embedded-NG 移植。
 *
 * 未直接移植热量限制和堵转阈值：这些参数依赖实车机构与裁判系统数据，当前
 * 工程尚不具备可靠标定条件。
 * 摩擦轮为 M3508/C620 直驱（gear_ratio=1），默认目标 25 rad/s。
 * 当前无就绪、卡弹回退、热量/裁判互锁或完整 FEEDING 状态机。
 */

#include "Shoot.h"
#include "../physical_units.h"
#include "board_config.h"

#include "message_center.h"

#if SHOOT
#include "dji_motor.h"
#include "fdcan.h"
#include <cmath>
#endif

#if SHOOT
static constexpr float SHOOT_DEFAULT_FRICTION_SPEED_RAD_S = 25.0f;
static constexpr float SHOOT_DEFAULT_RATE_HZ = 10.0f;
static constexpr float SHOOT_ONE_BULLET_ANGLE_RAD = DegToRad(36.0f);
static constexpr float SHOOT_REVERSE_SPEED_RAD_S = DegToRad(-360.0f);

#endif

namespace
{
struct ShootContext
{
    Subscriber<ShootCmd> command_subscriber{MessageCenter::Shoot_Command_Topic};
    Publisher<ShootFeedback> feedback_publisher{MessageCenter::Shoot_Feedback_Topic};
    ShootCmd command{};
    ShootFeedback feedback{};
    uint8_t feedback_divider = 0U;
#if SHOOT
    Class_DJIMotor friction_left;
    Class_DJIMotor friction_right;
    Class_DJIMotor loader;
    Struct_DJIMotor_Motion_Snapshot friction_left_snapshot;
    Struct_DJIMotor_Motion_Snapshot friction_right_snapshot;
    Struct_DJIMotor_Motion_Snapshot loader_snapshot;
    Class_DJIMotor_Group friction_group;
    Class_DJIMotor_Group loader_group;
    bool initialized = false;
    bool output_enabled = false;
    bool event_angle_active = false;
    float loader_angle_target_rad = 0.0f;
#endif
};

ShootContext ctx;
}

#if SHOOT

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
    if (enabled == ctx.output_enabled)
    {
        return;
    }
    ctx.output_enabled = enabled;
    if (enabled)
    {
        ctx.friction_group.Enable();
        ctx.loader_group.Enable();
    }
    else
    {
        ctx.friction_group.Disable();
        ctx.loader_group.Disable();
    }
}

static void Shoot_ApplyCommand(void)
{
    /* ShootMode 是总使能；关闭后摩擦轮和拨弹盘都停止主动输出。 */
    const bool enabled = ctx.command.shoot_mode == ShootMode::ON;
    Shoot_SetEnabled(enabled);
    if (!enabled)
    {
        ctx.event_angle_active = false;
        return;
    }

    float friction_reference_rad_s = 0.0f;
    if (ctx.command.friction_mode == FrictionMode::ON)
    {
        friction_reference_rad_s = ctx.command.friction_speed_rad_s > 0.0f
            ? ctx.command.friction_speed_rad_s
            : SHOOT_DEFAULT_FRICTION_SPEED_RAD_S;
    }
    ctx.friction_group.Control(friction_reference_rad_s, friction_reference_rad_s);

    float loader_speed_target_rad_s = 0.0f;
    switch (ctx.command.loader_mode)
    {
    case LoaderMode::BURST:
    {
        ctx.event_angle_active = false;
        ctx.loader.Set_Outer_Loop(DJI_MOTOR_SPEED_LOOP);
        const float rate = ctx.command.shoot_rate_hz > 0.0f
            ? ctx.command.shoot_rate_hz : SHOOT_DEFAULT_RATE_HZ;
        loader_speed_target_rad_s = ctx.command.loader_speed_rad_s != 0.0f
            ? ctx.command.loader_speed_rad_s
            : rate * SHOOT_ONE_BULLET_ANGLE_RAD;
        break;
    }

    case LoaderMode::REVERSE:
        ctx.event_angle_active = false;
        ctx.loader.Set_Outer_Loop(DJI_MOTOR_SPEED_LOOP);
        loader_speed_target_rad_s = ctx.command.loader_speed_rad_s != 0.0f
            ? -std::fabs(ctx.command.loader_speed_rad_s)
            : SHOOT_REVERSE_SPEED_RAD_S;
        break;

    case LoaderMode::STOP:
    default:
    {
        ShootEvent event;
        /* 每个 1 ms 周期最多取一个逻辑请求并累加目标角，不等待前一发物理完成。 */
        if (MessageCenter::Shoot_Event_Queue.Pop(event))
        {
            if (!ctx.event_angle_active)
            {
                ctx.loader_angle_target_rad =
                    ctx.loader_snapshot.output_total_angle;
            }
            const float bullet_count =
                event.type == ShootEventType::ShootTriple ? 3.0f : 1.0f;
            ctx.loader_angle_target_rad +=
                bullet_count * SHOOT_ONE_BULLET_ANGLE_RAD;
            ctx.event_angle_active = true;
        }
        if (ctx.event_angle_active)
        {
            ctx.loader.Set_Outer_Loop(DJI_MOTOR_ANGLE_LOOP);
        }
        else
        {
            ctx.loader.Set_Outer_Loop(DJI_MOTOR_SPEED_LOOP);
        }
        break;
    }
    }

    if (ctx.event_angle_active)
    {
        ctx.loader_group.Control(ctx.loader_angle_target_rad);
    }
    else
    {
        ctx.loader_group.Control(loader_speed_target_rad_s);
    }
}

static void Shoot_UpdateFeedback(void)
{
    ctx.feedback.friction_left_speed_rad_s =
        ctx.friction_left_snapshot.output_speed;
    ctx.feedback.friction_right_speed_rad_s =
        ctx.friction_right_snapshot.output_speed;
    ctx.feedback.loader_angle_rad = ctx.loader_snapshot.output_total_angle;
    ctx.feedback.loader_speed_rad_s = ctx.loader_snapshot.output_speed;
    ctx.feedback.enabled = ctx.output_enabled;
    ctx.feedback.online = ctx.friction_left_snapshot.online &&
                            ctx.friction_right_snapshot.online &&
                            ctx.loader_snapshot.online;
}
#endif

bool Shoot_Init(void)
{
    ctx.command = {};
    ctx.feedback = {};
    ctx.feedback_divider = 0U;

#if SHOOT
    Struct_DJIMotor_Init_Config friction_config{};
    friction_config.hfdcan = BoardConfig_Get().shoot_bus;
    friction_config.motor_type = Enum_DJIMotor_Type::M3508;
    friction_config.gear_ratio = 1.0f; // 摩擦轮直驱，不使用 M3508 默认减速比 19。
    friction_config.close_loop = DJI_MOTOR_SPEED_LOOP;
    friction_config.outer_loop = DJI_MOTOR_SPEED_LOOP;
    // 速度环输入为 rad/s；增益无可信实车标定依据，启用前需重新整定。
    friction_config.speed_pid = Shoot_MakePID(7.5f, 5.0f, 0.0f, 16000.0f, 16000.0f);

    friction_config.can_id = 3U;
    const bool left_initialized = ctx.friction_left.Init(friction_config);
    friction_config.can_id = 2U;
    friction_config.reverse = true;
    const bool right_initialized = ctx.friction_right.Init(friction_config);

    Struct_DJIMotor_Init_Config loader_config{};
    loader_config.hfdcan = BoardConfig_Get().shoot_bus;
    loader_config.can_id = 8U;
    loader_config.motor_type = Enum_DJIMotor_Type::M3508;
    loader_config.close_loop = DJI_MOTOR_CURRENT_LOOP |
                               DJI_MOTOR_SPEED_LOOP |
                               DJI_MOTOR_ANGLE_LOOP;
    loader_config.outer_loop = DJI_MOTOR_SPEED_LOOP;
    loader_config.current_pid = Shoot_MakePID(1.0f, 50.0f, 0.0f, 12000.0f, 12000.0f);
    loader_config.speed_pid = Shoot_MakePID(7.5f, 20.0f, 0.0f, 12000.0f, 12000.0f);
    // 角度环输出是 rad/s；原 360 deg/s 限幅转换为 2π rad/s。
    loader_config.angle_pid = Shoot_MakePID(10.0f, 0.0f, 0.0f,
                                            0.0f, DegToRad(360.0f));
    const bool loader_initialized = ctx.loader.Init(loader_config);

    ctx.initialized = left_initialized && right_initialized && loader_initialized &&
        ctx.friction_group.Init(&ctx.friction_left, &ctx.friction_right) &&
        ctx.loader_group.Init(&ctx.loader);
    ctx.output_enabled = true;
    if (ctx.initialized)
    {
        Shoot_SetEnabled(false);
    }
    ctx.event_angle_active = false;
    ctx.loader_angle_target_rad = 0.0f;
    return ctx.initialized;
#else
    return true;
#endif
}

void Shoot_Update(void)
{
    /* 每个控制周期读取最新命令；没有新消息时继续执行上一帧。 */
    ShootCmd command;
    if (ctx.command_subscriber.Read(command))
    {
        ctx.command = command;
    }

    if (ctx.command.shoot_mode == ShootMode::OFF)
    {
        ShootEvent discarded_event;
        size_t pending_events = MessageCenter::Shoot_Event_Queue.Size();
        while (pending_events-- > 0U &&
               MessageCenter::Shoot_Event_Queue.Pop(discarded_event))
        {
        }
    }

#if SHOOT
    if (ctx.initialized)
    {
        ctx.friction_left_snapshot = ctx.friction_left.GetMotionSnapshot();
        ctx.friction_right_snapshot = ctx.friction_right.GetMotionSnapshot();
        ctx.loader_snapshot = ctx.loader.GetMotionSnapshot();
        Shoot_ApplyCommand();
        Shoot_UpdateFeedback();
    }
#endif

    /* 控制按 1 kHz 更新，应用层反馈降频到 100 Hz。 */
    ctx.feedback_divider++;
    if (ctx.feedback_divider >= 10U)
    {
        ctx.feedback_divider = 0U;
        ctx.feedback_publisher.Publish(ctx.feedback);
    }
}

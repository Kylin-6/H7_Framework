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
#include "Shoot_Config.h"
#include "board_config.h"

#include "message_center.h"

#if SHOOT
#include "dji_motor.h"
#include "fdcan.h"
#include <cmath>
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
    bool event_angle_active = false;
    float loader_angle_target_rad = 0.0f;
#endif
};

ShootContext ctx;
}

#if SHOOT

static PID_InitTypeDef Shoot_MakePID(const ShootPidConfig &config)
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

static void Shoot_ApplyCommand(void)
{
    /* ShootMode 是总使能；关闭后摩擦轮和拨弹盘都停止主动输出。 */
    const bool enabled = ctx.command.shoot_mode == ShootMode::ON;
    (void)ctx.friction_group.RequestEnabled(enabled);
    (void)ctx.loader_group.RequestEnabled(enabled);
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
            : kShootConfig.default_friction_speed_rad_s;
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
            ? ctx.command.shoot_rate_hz : kShootConfig.default_rate_hz;
        loader_speed_target_rad_s = ctx.command.loader_speed_rad_s != 0.0f
            ? ctx.command.loader_speed_rad_s
            : rate * kShootConfig.one_bullet_angle_rad;
        break;
    }

    case LoaderMode::REVERSE:
        ctx.event_angle_active = false;
        ctx.loader.Set_Outer_Loop(DJI_MOTOR_SPEED_LOOP);
        loader_speed_target_rad_s = ctx.command.loader_speed_rad_s != 0.0f
            ? -std::fabs(ctx.command.loader_speed_rad_s)
            : kShootConfig.reverse_speed_rad_s;
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
                bullet_count * kShootConfig.one_bullet_angle_rad;
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
    ctx.feedback.enabled = ctx.command.shoot_mode == ShootMode::ON &&
                           ctx.friction_left_snapshot.ready &&
                           ctx.friction_right_snapshot.ready && ctx.loader_snapshot.ready;
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
    friction_config.gear_ratio = kShootConfig.friction_gear_ratio;
    friction_config.close_loop = DJI_MOTOR_SPEED_LOOP;
    friction_config.outer_loop = DJI_MOTOR_SPEED_LOOP;
    // 速度环输入为 rad/s；增益无可信实车标定依据，启用前需重新整定。
    friction_config.speed_pid = Shoot_MakePID(kShootConfig.friction_speed_pid);

    friction_config.can_id = kShootConfig.friction_left_id;
    const bool left_initialized = ctx.friction_left.Init(friction_config);
    friction_config.can_id = kShootConfig.friction_right_id;
    friction_config.reverse = true;
    const bool right_initialized = ctx.friction_right.Init(friction_config);

    Struct_DJIMotor_Init_Config loader_config{};
    loader_config.hfdcan = BoardConfig_Get().shoot_bus;
    loader_config.can_id = kShootConfig.loader_id;
    loader_config.motor_type = Enum_DJIMotor_Type::M3508;
    loader_config.close_loop = DJI_MOTOR_CURRENT_LOOP |
                               DJI_MOTOR_SPEED_LOOP |
                               DJI_MOTOR_ANGLE_LOOP;
    loader_config.outer_loop = DJI_MOTOR_SPEED_LOOP;
    loader_config.current_pid = Shoot_MakePID(kShootConfig.loader_current_pid);
    loader_config.speed_pid = Shoot_MakePID(kShootConfig.loader_speed_pid);
    // 角度环输出是 rad/s；原 360 deg/s 限幅转换为 2π rad/s。
    loader_config.angle_pid = Shoot_MakePID(kShootConfig.loader_angle_pid);
    const bool loader_initialized = ctx.loader.Init(loader_config);

    ctx.initialized = left_initialized && right_initialized && loader_initialized &&
        ctx.friction_group.Init(&ctx.friction_left, &ctx.friction_right) &&
        ctx.loader_group.Init(&ctx.loader);
    if (ctx.initialized)
    {
        (void)ctx.friction_group.RequestEnabled(false);
        (void)ctx.loader_group.RequestEnabled(false);
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

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
    bool event_angle_active = false; // 表示正在保持事件累加的角目标，不表示弹丸已完成发射。
    float loader_angle_target_rad = 0.0f;
#endif
};

ShootContext ctx;
} // namespace

#if SHOOT

/**
 * @brief 将应用 PID 参数转换为驱动配置，固定控制周期为 0.001 s。
 * @note 输出单位取决于环的位置：角度环为 rad/s，速度环为电流环目标或协议电流指令。
 */
static PID_InitTypeDef Shoot_MakePID(const ShootPidConfig& config)
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

/**
 * @brief 根据总开关、摩擦轮模式、拨弹模式和事件选择本周期目标并提交电机组。
 * @note 调用前已取得本周期反馈快照；事件仅累加目标角，不确认弹丸发射完成。
 *       驱动负责闭环和离线输出保护，本函数没有摩擦轮达速、热量或卡弹互锁。
 */
static void Shoot_ApplyCommand(void)
{
    /* ShootMode 是总使能；关闭后摩擦轮和拨弹盘都停止主动输出。 */
    const bool enabled = ctx.command.shoot_mode == ShootMode::ON;
    (void) ctx.friction_group.RequestEnabled(enabled);
    (void) ctx.loader_group.RequestEnabled(enabled);
    if (!enabled) // 总开关关闭，取消事件角度保持并禁止所有主动输出。
    {
        ctx.event_angle_active = false;
        return;
    }

    float friction_reference_rad_s = 0.0f; // Friction OFF 是使能状态下的零速度，不等于总开关 OFF。
    if (ctx.command.friction_mode == FrictionMode::ON) // 摩擦轮开启时采用正速度目标，否则保持零速度目标。
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
        // 连发以角速度控制，退出之前的事件角度保持；射速乘单弹角得到 rad/s。
        ctx.event_angle_active = false;
        const float rate = ctx.command.shoot_rate_hz > 0.0f
                               ? ctx.command.shoot_rate_hz
                               : kShootConfig.default_rate_hz;
        loader_speed_target_rad_s = ctx.command.loader_speed_rad_s != 0.0f
                                        ? ctx.command.loader_speed_rad_s
                                        : rate * kShootConfig.one_bullet_angle_rad;
        break;
    }

    case LoaderMode::REVERSE:
        ctx.event_angle_active = false;
        loader_speed_target_rad_s = ctx.command.loader_speed_rad_s != 0.0f
                                        ? -std::fabs(ctx.command.loader_speed_rad_s)
                                        : kShootConfig.reverse_speed_rad_s;
        break;

    case LoaderMode::STOP:
    default:
    {
        ShootEvent event;
        /* 每个 1 ms 周期最多取一个逻辑请求并累加目标角，不等待前一发物理完成。 */
        if (MessageCenter::Shoot_Event_Queue.Pop(event)) // STOP 模式有排队动作时，每周期只取一个请求。
        {
            if (!ctx.event_angle_active) // 还没有事件目标时，从当前拨弹盘累计角起步。
            {
                // 首次动作从当前反馈角起步；后续动作继续累加，避免覆盖排队的弹位。
                ctx.loader_angle_target_rad =
                    ctx.loader_snapshot.output_total_angle;
            }
            const float bullet_count =
                event.type == ShootEventType::ShootTriple ? 3.0f : 1.0f;
            ctx.loader_angle_target_rad +=
                bullet_count * kShootConfig.one_bullet_angle_rad;
            ctx.event_angle_active = true;
        }
        break;
    }
    }

    // 应用只选择目标和外环；角度/速度/电流 PID 及 CAN 发布复用 DJI 电机组接口。
    ctx.loader.Set_Outer_Loop(ctx.event_angle_active ? DJI_MOTOR_ANGLE_LOOP : DJI_MOTOR_SPEED_LOOP);
    ctx.loader_group.Control(ctx.event_angle_active ? ctx.loader_angle_target_rad : loader_speed_target_rad_s);
}

/**
 * @brief 从控制前取得的三个设备快照生成应用反馈，角度 rad、速度 rad/s。
 * @note enabled 表示总开关 ON 且三台电机 ready，不表示摩擦轮达速或完成射击。
 *       快照早于本周期使能请求，状态变化可能在下一周期反映。
 */
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

/**
 * @brief 清空应用消息状态，注册摩擦轮和拨弹盘并绑定各自的 DJI 发送组。
 * @return 硬件路径全部注册成功返回 true；SHOOT=0 时不注册设备并返回 true。
 * @note 由 ControlTask 启动时调用一次；总线归 BoardConfig，机构参数归 Shoot_Config。
 */
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
    friction_config.reverse = true; // 镜像安装：两侧使用同号逻辑速度，右侧由驱动反向。
    const bool right_initialized = ctx.friction_right.Init(friction_config);

    Struct_DJIMotor_Init_Config loader_config{};
    loader_config.hfdcan = BoardConfig_Get().shoot_bus;
    loader_config.can_id = kShootConfig.loader_id;
    loader_config.motor_type = Enum_DJIMotor_Type::M3508; // 未指定 gear_ratio，使用驱动默认 M3508 减速比 19。
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
    if (ctx.initialized) // 全部设备和发送组绑定成功后，建立禁止输出的初始请求。
    {
        (void) ctx.friction_group.RequestEnabled(false);
        (void) ctx.loader_group.RequestEnabled(false);
    }
    ctx.event_angle_active = false;
    ctx.loader_angle_target_rad = 0.0f;
    return ctx.initialized;
#else
    return true;
#endif
}

/**
 * @brief 1 kHz 入口：读取最新命令、清理禁用事件、执行控制并以 100 Hz 发布反馈。
 * @note Subscriber::Read 不检查消息年龄；未读到命令时保留上一帧，上层必须维护安全命令。
 */
void Shoot_Update(void)
{
    /* 每个控制周期读取最新命令；没有新消息时继续执行上一帧。 */
    ShootCmd command;
    if (ctx.command_subscriber.Read(command)) // Topic 有可读值时更新缓存，读取成功不代表命令仍新鲜。
    {
        ctx.command = command;
    }

    if (ctx.command.shoot_mode == ShootMode::OFF) // 禁用期间丢弃积压事件，避免重新开启后补射。
    {
        // 清除本周期开始时已有的事件，避免重新使能后补射；按队列快照限制循环次数。
        ShootEvent discarded_event;
        size_t pending_events = MessageCenter::Shoot_Event_Queue.Size();
        while (pending_events-- > 0U &&
               MessageCenter::Shoot_Event_Queue.Pop(discarded_event))
        {
        }
    }

#if SHOOT
    if (ctx.initialized) // 初始化成功才访问设备并运行硬件控制路径。
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
    if (ctx.feedback_divider >= 10U) // 每十个 1 ms 周期发布一次反馈。
    {
        ctx.feedback_divider = 0U;
        ctx.feedback_publisher.Publish(ctx.feedback);
    }
}

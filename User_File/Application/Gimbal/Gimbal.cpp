#include "Gimbal.h"
#include "message_center.h"
#if GIMBAL
#include "alg_pid.h"
#include "dmmotor.h"
#endif
#include <cmath>

namespace
{
constexpr uint64_t GIMBAL_INS_MAX_AGE_US = 10000U;
struct GimbalContext
{
    INS_State ins{};
    bool ins_valid = false;
    uint8_t feedback_divider = 0U;
#if GIMBAL
    Struct_Gimbal_Config config{};
    Class_DMMotor yaw_motor;
    Class_DMMotor pitch_motor;
    Class_PID yaw_angle_pid;
    Class_PID yaw_speed_pid;
    Struct_DMMotor_Snapshot yaw_snapshot{};
    Struct_DMMotor_Snapshot pitch_snapshot{};
    GimbalCmd command{};
    GimbalMode last_mode = GimbalMode::DISABLED;
    bool initialized = false;
    bool was_ready = false;
    float target_yaw_angle_rad = 0.0f;
    float target_pitch_angle_rad = 0.0f;
    float target_yaw_speed_rad_s = 0.0f;
    float target_pitch_speed_rad_s = 0.0f;
    bool yaw_registered = false;
    bool pitch_registered = false;
    uint32_t target_sequence = 0U;
#endif
};

GimbalContext ctx;

}

#if GIMBAL
namespace
{
constexpr float GIMBAL_PI = 3.14159265358979323846f;
float Clamp(float value, float minimum, float maximum)
{
    return value < minimum ? minimum : (value > maximum ? maximum : value);
}

bool ConfigValid(const Struct_Gimbal_Config &c)
{
    const float nonnegative[] = {c.yaw_angle_kp, c.yaw_speed_kp, c.yaw_speed_ki,
        c.yaw_speed_kd, c.yaw_integral_limit, c.pitch_kp, c.pitch_kd};
    for (float value : nonnegative)
    {
        if (!std::isfinite(value) || value < 0) { return false; }
    }
    const float positive[] = {c.yaw_speed_limit, c.yaw_torque_limit,
                              c.pitch_speed_limit, c.pitch_motor_per_imu};
    for (float value : positive)
    {
        if (!std::isfinite(value) || value <= 0) { return false; }
    }
    return !(c.yaw.bus == c.pitch.bus &&
             (c.yaw.id == c.pitch.id || c.yaw.feedback_id == c.pitch.feedback_id)) &&
           c.yaw_gyro_axis <= GimbalGyroAxis::Z && c.pitch_gyro_axis <= GimbalGyroAxis::Z &&
           (c.yaw_gyro_sign == 1 || c.yaw_gyro_sign == -1) &&
           (c.pitch_gyro_sign == 1 || c.pitch_gyro_sign == -1) &&
           c.yaw_integral_limit <= c.yaw_torque_limit &&
           std::isfinite(c.pitch_min) && std::isfinite(c.pitch_max) &&
           c.pitch_min < c.pitch_max;
}

float Gyro(GimbalGyroAxis axis, float sign)
{
    const float rates[] = {ctx.ins.gyro_x_rad_s, ctx.ins.gyro_y_rad_s,
                           ctx.ins.gyro_z_rad_s};
    return sign * rates[static_cast<unsigned>(axis)];
}

void ResetControllers()
{
    // PID::Init 保留历史，因此先重建值对象，清除积分、微分及目标历史。
    ctx.yaw_angle_pid = Class_PID{};
    ctx.yaw_speed_pid = Class_PID{};
    ctx.yaw_angle_pid.Init(ctx.config.yaw_angle_kp, 0, 0, 0, 0, ctx.config.yaw_speed_limit);
    ctx.yaw_speed_pid.Init(ctx.config.yaw_speed_kp, ctx.config.yaw_speed_ki,
        ctx.config.yaw_speed_kd, 0, ctx.config.yaw_integral_limit, ctx.config.yaw_torque_limit);
}

void CapturePose(uint32_t sequence)
{
    ResetControllers();
    ctx.target_yaw_angle_rad = ctx.ins.yaw_rad;
    ctx.target_pitch_angle_rad = ctx.ins.pitch_rad;
    ctx.target_yaw_speed_rad_s = ctx.target_pitch_speed_rad_s = 0;
    // 恢复前已发布的目标全部丢弃；IMU 只接受之后的新序号。
    ctx.target_sequence = sequence;
}

void Stop()
{
    // 首次失能请求或失能边沿立即尝试发布安全目标；补交与协议纠正交给 ServiceAll。
    if (ctx.yaw_registered) { (void)ctx.yaw_motor.RequestEnabled(false); }
    if (ctx.pitch_registered) { (void)ctx.pitch_motor.RequestEnabled(false); }
}

void Control(const Struct_DMMotor_Snapshot &pitch)
{
    // Yaw 复用现有 PID：最短角误差生成角速度，再由速度环生成转矩。
    const float error = std::remainder(ctx.target_yaw_angle_rad - ctx.ins.yaw_rad, 2 * GIMBAL_PI);
    ctx.yaw_angle_pid.Set_Target(error);
    ctx.yaw_angle_pid.Set_Now(0);
    ctx.yaw_angle_pid.TIM_Calculate_PeriodElapsedCallback();
    const float speed = ctx.yaw_angle_pid.Get_Out() + ctx.target_yaw_speed_rad_s;
    ctx.yaw_speed_pid.Set_Target(Clamp(speed, -ctx.config.yaw_speed_limit, ctx.config.yaw_speed_limit));
    ctx.yaw_speed_pid.Set_Now(Gyro(ctx.config.yaw_gyro_axis, ctx.config.yaw_gyro_sign));
    ctx.yaw_speed_pid.TIM_Calculate_PeriodElapsedCallback();
    const float torque = ctx.yaw_speed_pid.Get_Out();
    // Pitch 将 INS 姿态/角速度误差换算为电机目标；位置、速度均基于同一次反馈快照。
    const float position = pitch.feedback.position + ctx.config.pitch_motor_per_imu *
                          (ctx.target_pitch_angle_rad - ctx.ins.pitch_rad);
    const float velocity = pitch.feedback.velocity + ctx.config.pitch_motor_per_imu *
                          (ctx.target_pitch_speed_rad_s - Gyro(ctx.config.pitch_gyro_axis, ctx.config.pitch_gyro_sign));
    (void)ctx.yaw_motor.SetTorque(Clamp(torque, -ctx.config.yaw_torque_limit, ctx.config.yaw_torque_limit));
    (void)ctx.pitch_motor.SetMIT(Clamp(position, ctx.config.pitch_min, ctx.config.pitch_max),
        Clamp(velocity, -ctx.config.pitch_speed_limit, ctx.config.pitch_speed_limit), ctx.config.pitch_kp, ctx.config.pitch_kd, 0);
}

void UpdateTarget(const TopicSnapshot<GimbalCmd> &message)
{
    // LOCK 只在进入时捕获姿态；IMU 只接受新序号，避免恢复后重放旧目标。
    if (ctx.command.mode == GimbalMode::LOCK && ctx.last_mode != GimbalMode::LOCK)
    {
        CapturePose(message.sequence);
    }
    if (ctx.command.mode == GimbalMode::IMU && message.sequence != ctx.target_sequence)
    {
        ctx.target_yaw_angle_rad = ctx.command.yaw_angle_rad;
        ctx.target_pitch_angle_rad = ctx.command.pitch_angle_rad;
        ctx.target_yaw_speed_rad_s = ctx.command.yaw_speed_rad_s;
        ctx.target_pitch_speed_rad_s = ctx.command.pitch_speed_rad_s;
        ctx.target_sequence = message.sequence;
    }
    ctx.last_mode = ctx.command.mode;
}
} // namespace

bool Gimbal_Init(const Struct_Gimbal_Config &requested)
{
    if (!ConfigValid(requested))
    {
        ctx.initialized = false;
        return false;
    }
    ctx.config = requested;
    ctx.yaw_registered = ctx.yaw_motor.Init(ctx.config.yaw.bus, ctx.config.yaw.id, ctx.config.yaw.feedback_id,
        Enum_DMMotor_Mode::MIT, ctx.config.yaw.reverse, ctx.config.yaw.position_max,
        ctx.config.yaw.velocity_max, ctx.config.yaw.torque_max);
    ctx.pitch_registered = ctx.pitch_motor.Init(ctx.config.pitch.bus, ctx.config.pitch.id, ctx.config.pitch.feedback_id,
        Enum_DMMotor_Mode::MIT, ctx.config.pitch.reverse, ctx.config.pitch.position_max,
        ctx.config.pitch.velocity_max, ctx.config.pitch.torque_max);
    ctx.initialized = ctx.yaw_registered && ctx.pitch_registered;
    ctx.command = {};
    ctx.was_ready = false;
    ResetControllers();
    return ctx.initialized;
}

Enum_Gimbal_Status Gimbal_GetStatus(void)
{
    // 状态由当前输入与电机快照推导，仅用于观察，不安排重试或驱动状态迁移。
    if (!ctx.initialized) { return Gimbal_Status_CONFIG_ERROR; }
    if (ctx.command.mode == GimbalMode::DISABLED) { return Gimbal_Status_DISABLE; }
    if (!ctx.ins_valid || ctx.yaw_snapshot.fault || ctx.pitch_snapshot.fault)
    {
        return Gimbal_Status_FAULT;
    }
    return ctx.yaw_snapshot.ready && ctx.pitch_snapshot.ready
        ? Gimbal_Status_READY : Gimbal_Status_ENABLING;
}
#endif

static void PublishFeedback(void)
{
    if (++ctx.feedback_divider >= 10U)
    {
        ctx.feedback_divider = 0;
        GimbalFeedback feedback{};
        if (ctx.ins_valid)
        {
            feedback.yaw_rad = ctx.ins.yaw_rad;
            feedback.pitch_rad = ctx.ins.pitch_rad;
#if GIMBAL
            feedback.yaw_speed_rad_s = Gyro(ctx.config.yaw_gyro_axis, ctx.config.yaw_gyro_sign);
            feedback.pitch_speed_rad_s = Gyro(ctx.config.pitch_gyro_axis, ctx.config.pitch_gyro_sign);
#else
            feedback.yaw_speed_rad_s = ctx.ins.gyro_z_rad_s;
            feedback.pitch_speed_rad_s = ctx.ins.gyro_y_rad_s;
#endif
        }
        feedback.ins_valid = ctx.ins_valid;
#if GIMBAL
        feedback.enabled = ctx.yaw_snapshot.ready && ctx.pitch_snapshot.ready;
#endif
        MessageCenter::Gimbal_Feedback_Topic.Publish(feedback);
    }
}

void Gimbal_Update(void)
{
    ctx.ins_valid = MessageCenter::INS_State_Topic.ReadFresh(ctx.ins, GIMBAL_INS_MAX_AGE_US);
#if GIMBAL
    const auto message = MessageCenter::Gimbal_Command_Topic.ReadWithMeta();
    ctx.command = message.valid ? message.data : GimbalCmd{};
    ctx.yaw_snapshot = ctx.yaw_motor.GetFeedbackSnapshot();
    ctx.pitch_snapshot = ctx.pitch_motor.GetFeedbackSnapshot();

    if (!ctx.initialized || !message.valid ||
        ctx.command.mode == GimbalMode::DISABLED || !ctx.ins_valid ||
        ctx.yaw_snapshot.fault || ctx.pitch_snapshot.fault)
    {
        Stop();
        ctx.was_ready = false;
        ctx.last_mode = GimbalMode::DISABLED;
        ctx.yaw_snapshot = ctx.yaw_motor.GetFeedbackSnapshot();
        ctx.pitch_snapshot = ctx.pitch_motor.GetFeedbackSnapshot();
        PublishFeedback();
        return;
    }

    // 每周期表达输出许可；驱动只处理请求边沿，重复使能不会覆盖正常周期目标。
    (void)ctx.yaw_motor.RequestEnabled(true);
    (void)ctx.pitch_motor.RequestEnabled(true);
    ctx.yaw_snapshot = ctx.yaw_motor.GetFeedbackSnapshot();
    ctx.pitch_snapshot = ctx.pitch_motor.GetFeedbackSnapshot();
    if (!ctx.yaw_snapshot.ready || !ctx.pitch_snapshot.ready)
    {
        // 等待两轴反馈就绪；安全输出和低频协议纠正由 DMMotor 维护。
        ctx.was_ready = false;
        PublishFeedback();
        return;
    }

    if (!ctx.was_ready)
    {
        CapturePose(message.sequence);
        ctx.last_mode = ctx.command.mode;
        ctx.was_ready = true;
    }
    UpdateTarget(message);
    Control(ctx.pitch_snapshot);
#endif
    PublishFeedback();
}

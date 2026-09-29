#include "Gimbal.h"
#include "message_center.h"
#if GIMBAL
#include "alg_pid.h"
#include "dmmotor.h"
#include "sys_timestamp.h"
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
    Enum_Gimbal_Status status = Gimbal_Status_DISABLE;
    GimbalMode last_mode = GimbalMode::DISABLED;
    float target_yaw_angle_rad = 0.0f;
    float target_pitch_angle_rad = 0.0f;
    float target_yaw_speed_rad_s = 0.0f;
    float target_pitch_speed_rad_s = 0.0f;
    uint64_t state_since_us = 0U;
    uint64_t stable_since_us = 0U;
    uint64_t last_enable_us = 0U;
    uint64_t last_disable_us = 0U;
    bool yaw_registered = false;
    bool pitch_registered = false;
    bool stabilizing = false;
    bool enable_sent = false;
    bool disable_sent = false;
    uint32_t target_sequence = 0U;
#endif
};

GimbalContext ctx;

bool Gimbal_INS_Finite(const INS_State &ins)
{
    return std::isfinite(ins.yaw_rad) && std::isfinite(ins.pitch_rad) &&
           std::isfinite(ins.roll_rad) && std::isfinite(ins.gyro_x_rad_s) &&
           std::isfinite(ins.gyro_y_rad_s) && std::isfinite(ins.gyro_z_rad_s);
}
}

#if GIMBAL
namespace
{
constexpr float GIMBAL_PI = 3.14159265358979323846f;
constexpr uint64_t RETRY_US = 20000U;
constexpr uint64_t ENABLE_TIMEOUT_US = 2000000U;
constexpr uint64_t BACKOFF_US = 1000000U;
constexpr uint64_t STABLE_US = 100000U;
float Clamp(float value, float minimum, float maximum)
{
    return value < minimum ? minimum : (value > maximum ? maximum : value);
}

bool MotorConfigValid(const Struct_Gimbal_Motor_Config &motor)
{
    return (motor.bus == &hfdcan1 || motor.bus == &hfdcan2 || motor.bus == &hfdcan3) &&
           motor.id != 0 && motor.feedback_id <= 0x7ff &&
           std::isfinite(motor.position_max) && motor.position_max > 0 &&
           std::isfinite(motor.velocity_max) && motor.velocity_max > 0 &&
           std::isfinite(motor.torque_max) && motor.torque_max > 0;
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
    return MotorConfigValid(c.yaw) && MotorConfigValid(c.pitch) &&
           !(c.yaw.bus == c.pitch.bus &&
             (c.yaw.id == c.pitch.id || c.yaw.feedback_id == c.pitch.feedback_id)) &&
           c.yaw_gyro_axis <= GimbalGyroAxis::Z && c.pitch_gyro_axis <= GimbalGyroAxis::Z &&
           (c.yaw_gyro_sign == 1 || c.yaw_gyro_sign == -1) &&
           (c.pitch_gyro_sign == 1 || c.pitch_gyro_sign == -1) &&
           c.yaw_speed_limit <= c.yaw.velocity_max &&
           c.yaw_torque_limit <= c.yaw.torque_max &&
           c.yaw_integral_limit <= c.yaw_torque_limit &&
           c.pitch_kp <= 500 && c.pitch_kd <= 5 &&
           std::isfinite(c.pitch_min) && std::isfinite(c.pitch_max) &&
           c.pitch_min < c.pitch_max && c.pitch_min >= -c.pitch.position_max &&
           c.pitch_max <= c.pitch.position_max && c.pitch_speed_limit <= c.pitch.velocity_max;
}

bool CommandValid(const GimbalCmd &command)
{
    return (command.mode == GimbalMode::DISABLED || command.mode == GimbalMode::IMU ||
            command.mode == GimbalMode::LOCK) &&
           std::isfinite(command.yaw_angle_rad) && std::isfinite(command.pitch_angle_rad) &&
           std::isfinite(command.yaw_speed_rad_s) && std::isfinite(command.pitch_speed_rad_s);
}

bool FeedbackFinite(const Struct_DMMotor_Snapshot &snapshot)
{
    const auto &f = snapshot.feedback;
    return std::isfinite(f.position) && std::isfinite(f.total_position) &&
           std::isfinite(f.velocity) && std::isfinite(f.torque) &&
           std::isfinite(f.mos_temperature) && std::isfinite(f.rotor_temperature);
}

bool MotorFault(const Struct_DMMotor_Snapshot &snapshot)
{
    // 只有新鲜反馈中的状态才可用；0 为失能，1 为使能，其余状态不自动清错。
    return snapshot.online && snapshot.feedback.state > 1;
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

void SetState(Enum_Gimbal_Status state, uint64_t now)
{
    if (ctx.status != state)
    {
        ctx.status = state;
        ctx.state_since_us = now;
        ctx.stabilizing = false;
        ctx.enable_sent = ctx.disable_sent = false;
    }
}

bool ZeroOutput()
{
    // 两轴都尝试，不能用短路表达式跳过第二轴；覆盖尚未发出的旧周期帧。
    const bool yaw_ok = !ctx.yaw_registered || ctx.yaw_motor.SetTorque(0);
    const bool pitch_ok = !ctx.pitch_registered || ctx.pitch_motor.SetTorque(0);
    return yaw_ok && pitch_ok;
}

void Stop(uint64_t now, const Struct_DMMotor_Snapshot &yaw,
          const Struct_DMMotor_Snapshot &pitch)
{
    (void)ZeroOutput(); // 失败时下个周期继续覆盖，不将发布失败当作停机成功。
    if (!ctx.disable_sent || now - ctx.last_disable_us >= RETRY_US)
    {
        if (ctx.yaw_registered && (!yaw.online || yaw.feedback.state != 0))
        {
            (void)ctx.yaw_motor.Disable();
        }
        if (ctx.pitch_registered && (!pitch.online || pitch.feedback.state != 0))
        {
            (void)ctx.pitch_motor.Disable();
        }
        ctx.last_disable_us = now;
        ctx.disable_sent = true;
    }
}

bool Control(const Struct_DMMotor_Snapshot &pitch)
{
    const float error = std::remainder(ctx.target_yaw_angle_rad - ctx.ins.yaw_rad, 2 * GIMBAL_PI);
    if (!std::isfinite(error)) { return false; }
    ctx.yaw_angle_pid.Set_Target(error);
    ctx.yaw_angle_pid.Set_Now(0);
    ctx.yaw_angle_pid.TIM_Calculate_PeriodElapsedCallback();
    const float speed = ctx.yaw_angle_pid.Get_Out() + ctx.target_yaw_speed_rad_s;
    if (!std::isfinite(speed)) { return false; }
    ctx.yaw_speed_pid.Set_Target(Clamp(speed, -ctx.config.yaw_speed_limit, ctx.config.yaw_speed_limit));
    ctx.yaw_speed_pid.Set_Now(Gyro(ctx.config.yaw_gyro_axis, ctx.config.yaw_gyro_sign));
    ctx.yaw_speed_pid.TIM_Calculate_PeriodElapsedCallback();
    const float torque = ctx.yaw_speed_pid.Get_Out();
    const float position = pitch.feedback.position + ctx.config.pitch_motor_per_imu *
                          (ctx.target_pitch_angle_rad - ctx.ins.pitch_rad);
    const float velocity = pitch.feedback.velocity + ctx.config.pitch_motor_per_imu *
                          (ctx.target_pitch_speed_rad_s - Gyro(ctx.config.pitch_gyro_axis, ctx.config.pitch_gyro_sign));
    if (!std::isfinite(torque) || !std::isfinite(position) || !std::isfinite(velocity)) { return false; }
    const bool yaw_ok = ctx.yaw_motor.SetTorque(Clamp(torque, -ctx.config.yaw_torque_limit, ctx.config.yaw_torque_limit));
    const bool pitch_ok = ctx.pitch_motor.SetMIT(Clamp(position, ctx.config.pitch_min, ctx.config.pitch_max),
        Clamp(velocity, -ctx.config.pitch_speed_limit, ctx.config.pitch_speed_limit), ctx.config.pitch_kp, ctx.config.pitch_kd, 0);
    return yaw_ok && pitch_ok;
}

void UpdateControl(const TopicSnapshot<GimbalCmd> &message,
                   const Struct_DMMotor_Snapshot &yaw, const Struct_DMMotor_Snapshot &pitch)
{
    const uint64_t now = SYS_Timestamp_Get_Microsecond();
    if (ctx.status == Gimbal_Status_CONFIG_ERROR)
    {
        Stop(now, yaw, pitch);
        return;
    }
    const GimbalCmd command = message.valid ? message.data : GimbalCmd{};
    const bool valid = CommandValid(command) && ctx.ins_valid &&
                       FeedbackFinite(yaw) && FeedbackFinite(pitch) &&
                       !MotorFault(yaw) && !MotorFault(pitch);
    const bool healthy = yaw.online && yaw.enabled && pitch.online && pitch.enabled;
    if (command.mode == GimbalMode::DISABLED)
    {
        SetState(Gimbal_Status_DISABLE, now);
        Stop(now, yaw, pitch);
        ctx.last_mode = GimbalMode::DISABLED;
        return;
    }
    if (!valid || (ctx.status == Gimbal_Status_READY && !healthy))
    {
        SetState(Gimbal_Status_FAULT, now);
    }
    if (ctx.status == Gimbal_Status_FAULT)
    {
        Stop(now, yaw, pitch);
        if (valid && now - ctx.state_since_us >= BACKOFF_US)
        {
            SetState(Gimbal_Status_ENABLING, now);
        }
        return;
    }
    if (ctx.status == Gimbal_Status_DISABLE)
    {
        SetState(Gimbal_Status_ENABLING, now);
    }
    if (ctx.status == Gimbal_Status_ENABLING)
    {
        if (!ZeroOutput() || now - ctx.state_since_us >= ENABLE_TIMEOUT_US)
        {
            SetState(Gimbal_Status_FAULT, now);
            Stop(now, yaw, pitch);
            return;
        }
        if (!ctx.enable_sent || now - ctx.last_enable_us >= RETRY_US)
        {
            if (!yaw.online || !yaw.enabled) { (void)ctx.yaw_motor.Enable(); }
            if (!pitch.online || !pitch.enabled) { (void)ctx.pitch_motor.Enable(); }
            ctx.last_enable_us = now;
            ctx.enable_sent = true;
        }
        if (!healthy) { ctx.stabilizing = false; }
        else if (!ctx.stabilizing) { ctx.stable_since_us = now; ctx.stabilizing = true; }
        else if (now - ctx.stable_since_us >= STABLE_US)
        {
            CapturePose(message.sequence);
            ctx.last_mode = command.mode;
            SetState(Gimbal_Status_READY, now);
        }
        return;
    }
    if (command.mode == GimbalMode::LOCK && ctx.last_mode != GimbalMode::LOCK)
    {
        CapturePose(message.sequence);
    }
    if (command.mode == GimbalMode::IMU && message.sequence != ctx.target_sequence)
    {
        ctx.target_yaw_angle_rad = command.yaw_angle_rad;
        ctx.target_pitch_angle_rad = command.pitch_angle_rad;
        ctx.target_yaw_speed_rad_s = command.yaw_speed_rad_s;
        ctx.target_pitch_speed_rad_s = command.pitch_speed_rad_s;
        ctx.target_sequence = message.sequence;
    }
    ctx.last_mode = command.mode;
    if (!Control(pitch))
    {
        SetState(Gimbal_Status_FAULT, now);
        Stop(now, yaw, pitch);
    }
}
} // namespace

bool Gimbal_Init(const Struct_Gimbal_Config &requested)
{
    if (!ConfigValid(requested))
    {
        ctx.status = Gimbal_Status_CONFIG_ERROR;
        return false;
    }
    ctx.config = requested;
    ctx.yaw_motor.SetAutoEnableOnOffline(false);
    ctx.pitch_motor.SetAutoEnableOnOffline(false);
    ctx.yaw_registered = ctx.yaw_motor.Init(ctx.config.yaw.bus, ctx.config.yaw.id, ctx.config.yaw.feedback_id,
        Enum_DMMotor_Mode::MIT, ctx.config.yaw.reverse, ctx.config.yaw.position_max,
        ctx.config.yaw.velocity_max, ctx.config.yaw.torque_max);
    ctx.pitch_registered = ctx.pitch_motor.Init(ctx.config.pitch.bus, ctx.config.pitch.id, ctx.config.pitch.feedback_id,
        Enum_DMMotor_Mode::MIT, ctx.config.pitch.reverse, ctx.config.pitch.position_max,
        ctx.config.pitch.velocity_max, ctx.config.pitch.torque_max);
    ctx.status = ctx.yaw_registered && ctx.pitch_registered ? Gimbal_Status_DISABLE : Gimbal_Status_CONFIG_ERROR;
    ResetControllers();
    return ctx.yaw_registered && ctx.pitch_registered;
}

Enum_Gimbal_Status Gimbal_GetStatus(void)
{
    return ctx.status;
}
#endif

void Gimbal_Update(void)
{
    ctx.ins_valid = MessageCenter::INS_State_Topic.ReadFresh(ctx.ins, GIMBAL_INS_MAX_AGE_US) &&
                       Gimbal_INS_Finite(ctx.ins);
#if GIMBAL
    ctx.yaw_snapshot = ctx.yaw_motor.GetFeedbackSnapshot();
    ctx.pitch_snapshot = ctx.pitch_motor.GetFeedbackSnapshot();
    UpdateControl(MessageCenter::Gimbal_Command_Topic.ReadWithMeta(),
                  ctx.yaw_snapshot, ctx.pitch_snapshot);
#endif
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
        feedback.enabled = ctx.yaw_snapshot.online && ctx.yaw_snapshot.enabled &&
                           ctx.pitch_snapshot.online && ctx.pitch_snapshot.enabled;
#endif
        MessageCenter::Gimbal_Feedback_Topic.Publish(feedback);
    }
}

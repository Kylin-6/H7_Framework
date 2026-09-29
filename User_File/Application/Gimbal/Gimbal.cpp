#include "Gimbal.h"
#include "message_center.h"
#include <cmath>

namespace
{
INS_State Gimbal_INS_State;
bool Gimbal_INS_Valid = false;
constexpr uint64_t GIMBAL_INS_MAX_AGE_US = 10000U;
uint8_t Gimbal_Feedback_Divider;

bool Gimbal_INS_Finite(const INS_State &ins)
{
    return std::isfinite(ins.yaw_rad) && std::isfinite(ins.pitch_rad) &&
           std::isfinite(ins.roll_rad) && std::isfinite(ins.gyro_x_rad_s) &&
           std::isfinite(ins.gyro_y_rad_s) && std::isfinite(ins.gyro_z_rad_s);
}
}

#if GIMBAL
#include "sys_timestamp.h"

Struct_Gimbal Gimbal;

namespace
{
constexpr float GIMBAL_PI = 3.14159265358979323846f;
constexpr uint64_t RETRY_US = 20000U;
constexpr uint64_t ENABLE_TIMEOUT_US = 2000000U;
constexpr uint64_t BACKOFF_US = 1000000U;
constexpr uint64_t STABLE_US = 100000U;
Struct_Gimbal_Config config;
bool yaw_registered, pitch_registered;
GimbalMode last_mode = GimbalMode::DISABLED;
uint64_t state_since, stable_since, last_enable, last_disable;
bool stabilizing, enable_sent, disable_sent;
uint32_t target_sequence;

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
    const float rates[] = {Gimbal_INS_State.gyro_x_rad_s, Gimbal_INS_State.gyro_y_rad_s,
                           Gimbal_INS_State.gyro_z_rad_s};
    return sign * rates[static_cast<unsigned>(axis)];
}

void ResetControllers()
{
    // PID::Init 保留历史，因此先重建值对象，清除积分、微分及目标历史。
    Gimbal.Yaw_Angle_PID = Class_PID{};
    Gimbal.Yaw_Speed_PID = Class_PID{};
    Gimbal.Yaw_Angle_PID.Init(config.yaw_angle_kp, 0, 0, 0, 0, config.yaw_speed_limit);
    Gimbal.Yaw_Speed_PID.Init(config.yaw_speed_kp, config.yaw_speed_ki,
        config.yaw_speed_kd, 0, config.yaw_integral_limit, config.yaw_torque_limit);
}

void CapturePose(uint32_t sequence)
{
    ResetControllers();
    Gimbal.Target_Yaw_Angle = Gimbal_INS_State.yaw_rad;
    Gimbal.Target_Pitch_Angle = Gimbal_INS_State.pitch_rad;
    Gimbal.Target_Yaw_Speed = Gimbal.Target_Pitch_Speed = 0;
    // 恢复前已发布的目标全部丢弃；IMU 只接受之后的新序号。
    target_sequence = sequence;
}

void SetState(Enum_Gimbal_Status state, uint64_t now)
{
    if (Gimbal.status != state)
    {
        Gimbal.status = state;
        state_since = now;
        stabilizing = false;
        enable_sent = disable_sent = false;
    }
}

bool ZeroOutput()
{
    // 两轴都尝试，不能用短路表达式跳过第二轴；覆盖尚未发出的旧周期帧。
    const bool yaw_ok = !yaw_registered || Gimbal.Yaw_Motor.SetTorque(0);
    const bool pitch_ok = !pitch_registered || Gimbal.Pitch_Motor.SetTorque(0);
    return yaw_ok && pitch_ok;
}

void Stop(uint64_t now, const Struct_DMMotor_Snapshot &yaw,
          const Struct_DMMotor_Snapshot &pitch)
{
    (void)ZeroOutput(); // 失败时下个周期继续覆盖，不将发布失败当作停机成功。
    if (!disable_sent || now - last_disable >= RETRY_US)
    {
        if (yaw_registered && (!yaw.online || yaw.feedback.state != 0))
        {
            (void)Gimbal.Yaw_Motor.Disable();
        }
        if (pitch_registered && (!pitch.online || pitch.feedback.state != 0))
        {
            (void)Gimbal.Pitch_Motor.Disable();
        }
        last_disable = now;
        disable_sent = true;
    }
}

bool Control(const Struct_DMMotor_Snapshot &pitch)
{
    const float error = std::remainder(Gimbal.Target_Yaw_Angle - Gimbal_INS_State.yaw_rad, 2 * GIMBAL_PI);
    if (!std::isfinite(error)) { return false; }
    Gimbal.Yaw_Angle_PID.Set_Target(error);
    Gimbal.Yaw_Angle_PID.Set_Now(0);
    Gimbal.Yaw_Angle_PID.TIM_Calculate_PeriodElapsedCallback();
    const float speed = Gimbal.Yaw_Angle_PID.Get_Out() + Gimbal.Target_Yaw_Speed;
    if (!std::isfinite(speed)) { return false; }
    Gimbal.Yaw_Speed_PID.Set_Target(Clamp(speed, -config.yaw_speed_limit, config.yaw_speed_limit));
    Gimbal.Yaw_Speed_PID.Set_Now(Gyro(config.yaw_gyro_axis, config.yaw_gyro_sign));
    Gimbal.Yaw_Speed_PID.TIM_Calculate_PeriodElapsedCallback();
    const float torque = Gimbal.Yaw_Speed_PID.Get_Out();
    const float position = pitch.feedback.position + config.pitch_motor_per_imu *
                          (Gimbal.Target_Pitch_Angle - Gimbal_INS_State.pitch_rad);
    const float velocity = pitch.feedback.velocity + config.pitch_motor_per_imu *
                          (Gimbal.Target_Pitch_Speed - Gyro(config.pitch_gyro_axis, config.pitch_gyro_sign));
    if (!std::isfinite(torque) || !std::isfinite(position) || !std::isfinite(velocity)) { return false; }
    const bool yaw_ok = Gimbal.Yaw_Motor.SetTorque(Clamp(torque, -config.yaw_torque_limit, config.yaw_torque_limit));
    const bool pitch_ok = Gimbal.Pitch_Motor.SetMIT(Clamp(position, config.pitch_min, config.pitch_max),
        Clamp(velocity, -config.pitch_speed_limit, config.pitch_speed_limit), config.pitch_kp, config.pitch_kd, 0);
    return yaw_ok && pitch_ok;
}

void UpdateControl(const TopicSnapshot<GimbalCmd> &message,
                   const Struct_DMMotor_Snapshot &yaw, const Struct_DMMotor_Snapshot &pitch)
{
    const uint64_t now = SYS_Timestamp_Get_Microsecond();
    if (Gimbal.status == Gimbal_Status_CONFIG_ERROR)
    {
        Stop(now, yaw, pitch);
        return;
    }
    const GimbalCmd command = message.valid ? message.data : GimbalCmd{};
    const bool valid = CommandValid(command) && Gimbal_INS_Valid &&
                       FeedbackFinite(yaw) && FeedbackFinite(pitch) &&
                       !MotorFault(yaw) && !MotorFault(pitch);
    const bool healthy = yaw.online && yaw.enabled && pitch.online && pitch.enabled;
    if (command.mode == GimbalMode::DISABLED)
    {
        SetState(Gimbal_Status_DISABLE, now);
        Stop(now, yaw, pitch);
        last_mode = GimbalMode::DISABLED;
        return;
    }
    if (!valid || (Gimbal.status == Gimbal_Status_READY && !healthy))
    {
        SetState(Gimbal_Status_FAULT, now);
    }
    if (Gimbal.status == Gimbal_Status_FAULT)
    {
        Stop(now, yaw, pitch);
        if (valid && now - state_since >= BACKOFF_US)
        {
            SetState(Gimbal_Status_ENABLING, now);
        }
        return;
    }
    if (Gimbal.status == Gimbal_Status_DISABLE)
    {
        SetState(Gimbal_Status_ENABLING, now);
    }
    if (Gimbal.status == Gimbal_Status_ENABLING)
    {
        if (!ZeroOutput() || now - state_since >= ENABLE_TIMEOUT_US)
        {
            SetState(Gimbal_Status_FAULT, now);
            Stop(now, yaw, pitch);
            return;
        }
        if (!enable_sent || now - last_enable >= RETRY_US)
        {
            if (!yaw.online || !yaw.enabled) { (void)Gimbal.Yaw_Motor.Enable(); }
            if (!pitch.online || !pitch.enabled) { (void)Gimbal.Pitch_Motor.Enable(); }
            last_enable = now;
            enable_sent = true;
        }
        if (!healthy) { stabilizing = false; }
        else if (!stabilizing) { stable_since = now; stabilizing = true; }
        else if (now - stable_since >= STABLE_US)
        {
            CapturePose(message.sequence);
            last_mode = command.mode;
            SetState(Gimbal_Status_READY, now);
        }
        return;
    }
    if (command.mode == GimbalMode::LOCK && last_mode != GimbalMode::LOCK)
    {
        CapturePose(message.sequence);
    }
    if (command.mode == GimbalMode::IMU && message.sequence != target_sequence)
    {
        Gimbal.Target_Yaw_Angle = command.yaw_angle_rad;
        Gimbal.Target_Pitch_Angle = command.pitch_angle_rad;
        Gimbal.Target_Yaw_Speed = command.yaw_speed_rad_s;
        Gimbal.Target_Pitch_Speed = command.pitch_speed_rad_s;
        target_sequence = message.sequence;
    }
    last_mode = command.mode;
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
        Gimbal.status = Gimbal_Status_CONFIG_ERROR;
        return false;
    }
    config = requested;
    Gimbal.Yaw_Motor.SetAutoEnableOnOffline(false);
    Gimbal.Pitch_Motor.SetAutoEnableOnOffline(false);
    yaw_registered = Gimbal.Yaw_Motor.Init(config.yaw.bus, config.yaw.id, config.yaw.feedback_id,
        Enum_DMMotor_Mode::MIT, config.yaw.reverse, config.yaw.position_max,
        config.yaw.velocity_max, config.yaw.torque_max);
    pitch_registered = Gimbal.Pitch_Motor.Init(config.pitch.bus, config.pitch.id, config.pitch.feedback_id,
        Enum_DMMotor_Mode::MIT, config.pitch.reverse, config.pitch.position_max,
        config.pitch.velocity_max, config.pitch.torque_max);
    Gimbal.status = yaw_registered && pitch_registered ? Gimbal_Status_DISABLE : Gimbal_Status_CONFIG_ERROR;
    ResetControllers();
    return yaw_registered && pitch_registered;
}
#endif

void Gimbal_Update(void)
{
    Gimbal_INS_Valid = MessageCenter::INS_State_Topic.ReadFresh(Gimbal_INS_State, GIMBAL_INS_MAX_AGE_US) &&
                       Gimbal_INS_Finite(Gimbal_INS_State);
#if GIMBAL
    const auto yaw = Gimbal.Yaw_Motor.GetFeedbackSnapshot();
    const auto pitch = Gimbal.Pitch_Motor.GetFeedbackSnapshot();
    UpdateControl(MessageCenter::Gimbal_Command_Topic.ReadWithMeta(), yaw, pitch);
#endif
    if (++Gimbal_Feedback_Divider >= 10U)
    {
        Gimbal_Feedback_Divider = 0;
        GimbalFeedback feedback{};
        if (Gimbal_INS_Valid)
        {
            feedback.yaw_rad = Gimbal_INS_State.yaw_rad;
            feedback.pitch_rad = Gimbal_INS_State.pitch_rad;
#if GIMBAL
            feedback.yaw_speed_rad_s = Gyro(config.yaw_gyro_axis, config.yaw_gyro_sign);
            feedback.pitch_speed_rad_s = Gyro(config.pitch_gyro_axis, config.pitch_gyro_sign);
#else
            feedback.yaw_speed_rad_s = Gimbal_INS_State.gyro_z_rad_s;
            feedback.pitch_speed_rad_s = Gimbal_INS_State.gyro_y_rad_s;
#endif
        }
        feedback.ins_valid = Gimbal_INS_Valid;
#if GIMBAL
        feedback.enabled = yaw.online && yaw.enabled && pitch.online && pitch.enabled;
#endif
        MessageCenter::Gimbal_Feedback_Topic.Publish(feedback);
    }
}

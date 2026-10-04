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

} // namespace

#if GIMBAL
namespace
{
constexpr float GIMBAL_PI = 3.14159265358979323846f;
/**
 * @brief 将数值限制在闭区间 [minimum, maximum] 内。
 * @note 调用方保证上下界有序、输入有限；不承担参数校验。
 */
float Clamp(float value, float minimum, float maximum)
{
    return value < minimum ? minimum : (value > maximum ? maximum : value);
}

/**
 * @brief 检查应用控制参数、IMU 轴/符号及同总线两轴 ID 冲突。
 * @return 应用层约束全部满足时返回 true；总线、协议量程等由设备 Init 继续校验。
 */
bool ConfigValid(const Struct_Gimbal_Config& c)
{
    const float nonnegative[] = {c.yaw_angle_kp, c.yaw_speed_kp, c.yaw_speed_ki,
                                 c.yaw_speed_kd, c.yaw_integral_limit, c.pitch_kp, c.pitch_kd};
    for (float value : nonnegative)
    {
        if (!std::isfinite(value) || value < 0)
        {
            return false;
        }
    }
    const float positive[] = {c.yaw_speed_limit, c.yaw_torque_limit,
                              c.pitch_speed_limit, c.pitch_motor_per_imu};
    for (float value : positive)
    {
        if (!std::isfinite(value) || value <= 0)
        {
            return false;
        }
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

/**
 * @brief 从本周期 INS 快照提取指定机体系角速度，并乘方向符号，单位 rad/s。
 * @param axis 已由配置校验的 X/Y/Z 轴。
 * @param sign 安装方向符号，只允许 +1 或 -1。
 * @note 不执行完整姿态坐标变换；控制调用方须先确认 INS 新鲜度。
 */
float Gyro(GimbalGyroAxis axis, float sign)
{
    const float rates[] = {ctx.ins.gyro_x_rad_s, ctx.ins.gyro_y_rad_s,
                           ctx.ins.gyro_z_rad_s};
    return sign * rates[static_cast<unsigned>(axis)];
}

/**
 * @brief 清空 Yaw 两级 PID 的历史状态，再按当前配置设置增益和输出限幅。
 * @note 初始化、捕获姿态和恢复时调用；Pitch 的 MIT 增益直接随指令提交。
 */
void ResetControllers()
{
    // PID::Init 保留历史，因此先重建值对象，清除积分、微分及目标历史。
    ctx.yaw_angle_pid = Class_PID{};
    ctx.yaw_speed_pid = Class_PID{};
    ctx.yaw_angle_pid.Init(ctx.config.yaw_angle_kp, 0, 0, 0, 0, ctx.config.yaw_speed_limit);
    ctx.yaw_speed_pid.Init(ctx.config.yaw_speed_kp, ctx.config.yaw_speed_ki,
                           ctx.config.yaw_speed_kd, 0, ctx.config.yaw_integral_limit, ctx.config.yaw_torque_limit);
}

/**
 * @brief 以当前有效 INS 姿态建立保持目标，清空控制器历史及速度前馈。
 * @param sequence 当前命令发布序号，用于拒绝恢复前的旧 IMU 目标。
 * @note 调用前须确认 INS 有效；不设置电机机械零位。
 */
void CapturePose(uint32_t sequence)
{
    ResetControllers();
    ctx.target_yaw_angle_rad = ctx.ins.yaw_rad;
    ctx.target_pitch_angle_rad = ctx.ins.pitch_rad;
    ctx.target_yaw_speed_rad_s = ctx.target_pitch_speed_rad_s = 0;
    ctx.target_sequence = sequence;
}

/**
 * @brief 对已经注册成功的电机表达云台功能许可，允许部分初始化失败时调用。
 * @note 不等待协议确认；安全目标提交和后续协议补交由 DMMotor 处理。
 */
void SetEnabled(bool enabled)
{
    if (ctx.yaw_registered)
    {
        (void) ctx.yaw_motor.RequestEnabled(enabled);
    }
    if (ctx.pitch_registered)
    {
        (void) ctx.pitch_motor.RequestEnabled(enabled);
    }
}

/**
 * @brief 计算双轴控制并提交限幅后的电机目标。
 * @param pitch 本周期 Pitch 快照，位置 rad、速度 rad/s，方向已由驱动统一。
 * @note 仅在云台功能获许可并更新目标后调用，不等待 CAN 发送或电机执行。
 */
void Control(const Struct_DMMotor_Snapshot& pitch)
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
    (void) ctx.yaw_motor.SetTorque(Clamp(torque, -ctx.config.yaw_torque_limit, ctx.config.yaw_torque_limit));
    (void) ctx.pitch_motor.SetMIT(Clamp(position, ctx.config.pitch_min, ctx.config.pitch_max),
                                  Clamp(velocity, -ctx.config.pitch_speed_limit, ctx.config.pitch_speed_limit), ctx.config.pitch_kp, ctx.config.pitch_kd, 0);
}

/**
 * @brief 根据模式切换和发布序号更新私有控制目标。
 * @param message 与 ctx.command 对应的有效命令快照；这里只使用其 sequence。
 * @note 未就绪期间保持当前姿态目标；首次就绪或恢复时重新捕获，IMU 只接收随后新序号。
 */
void UpdateTarget(const TopicSnapshot<GimbalCmd>& message)
{
    const bool ready = ctx.yaw_snapshot.ready && ctx.pitch_snapshot.ready;
    if (!ready || !ctx.was_ready)
    {
        // ready 只决定姿态捕获时机；未就绪电机的输出由驱动安全化。
        CapturePose(message.sequence);
        ctx.last_mode = ctx.command.mode;
    }
    ctx.was_ready = ready;
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

/**
 * @brief 启动阶段校验并复制配置、注册两轴 MIT 电机、初始化控制器。
 * @param requested 参数配置，复制后调用方无需保留其对象。
 * @return 配置有效且两轴注册成功时返回 true；失败后更新入口禁止正常控制。
 * @note 同一 ControlTask 启动时仅调用一次；不等待反馈、不使能、不置零，
 *       也不修改电机端模式或持久化参数。
 */
bool Gimbal_Init(const Struct_Gimbal_Config& requested)
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

/**
 * @brief 根据最近一次更新留下的命令、INS 和电机快照推导应用状态。
 * @note 由同一任务上下文读取；不刷新设备快照、不执行控制或安排恢复。
 */
Enum_Gimbal_Status Gimbal_GetStatus(void)
{
    if (!ctx.initialized)
    {
        return Gimbal_Status_CONFIG_ERROR;
    }
    if (ctx.command.mode == GimbalMode::DISABLED)
    {
        return Gimbal_Status_DISABLE;
    }
    if (!ctx.ins_valid || ctx.yaw_snapshot.fault || ctx.pitch_snapshot.fault)
    {
        return Gimbal_Status_FAULT;
    }
    return ctx.yaw_snapshot.ready && ctx.pitch_snapshot.ready
               ? Gimbal_Status_READY
               : Gimbal_Status_ENABLING;
}
#endif

/**
 * @brief 每调用十次发布一次云台反馈；1 kHz 更新入口下对应 100 Hz。
 * @note 姿态来自 INS，INS 无效时姿态/速度为零；enabled 表示功能获许可且两轴 ready。
 *       关闭硬件路径时仍发布 INS 反馈，enabled 保持 false。
 */
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
        feedback.enabled = ctx.was_ready;
#endif
        MessageCenter::Gimbal_Feedback_Topic.Publish(feedback);
    }
}

/**
 * @brief ControlTask 的 1 kHz 周期入口：读取快照、处理许可/恢复、更新目标和输出。
 * @note 在 RobotCmd_Update 之后调用；禁用或 INS 无效时请求功能停机，
 *       设备故障和掉线输出由驱动保护，各路径均维护反馈分频。
 *       GIMBAL=0 时仅更新 INS 反馈；本入口不阻塞、不解析 CAN、不仲裁命令来源。
 */
void Gimbal_Update(void)
{
    ctx.ins_valid = MessageCenter::INS_State_Topic.ReadFresh(ctx.ins, GIMBAL_INS_MAX_AGE_US);
#if GIMBAL
    const auto message = MessageCenter::Gimbal_Command_Topic.ReadWithMeta();
    ctx.command = message.valid ? message.data : GimbalCmd{};
    ctx.yaw_snapshot = ctx.yaw_motor.GetFeedbackSnapshot();
    ctx.pitch_snapshot = ctx.pitch_motor.GetFeedbackSnapshot();

    const bool enabled = ctx.initialized && ctx.command.mode != GimbalMode::DISABLED && ctx.ins_valid;
    SetEnabled(enabled);
    if (enabled)
    {
        UpdateTarget(message);
        Control(ctx.pitch_snapshot);
    }
    else
    {
        ctx.was_ready = false;
        ctx.last_mode = GimbalMode::DISABLED;
    }
#endif
    PublishFeedback();
}

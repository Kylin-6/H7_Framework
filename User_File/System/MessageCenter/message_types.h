#ifndef MESSAGE_TYPES_H
#define MESSAGE_TYPES_H

#include <cstdint>

enum class ChassisMode : uint8_t
{
    ZERO_FORCE = 0,
    NO_FOLLOW,
    FOLLOW_GIMBAL_YAW,
    ROTATE,
};

enum class GimbalMode : uint8_t
{
    DISABLED = 0,
    IMU,
    LOCK,
};

enum class ShootMode : uint8_t
{
    OFF = 0,
    ON,
};

enum class FrictionMode : uint8_t
{
    OFF = 0,
    ON,
};

/** 拨弹盘的连续工作模式；单发和三连发由 ShootEvent 表示。 */
enum class LoaderMode : uint8_t
{
    STOP = 0,
    REVERSE,
    BURST,
};

enum class ShootEventType : uint8_t
{
    ShootOnce = 0,
    ShootTriple,
};

struct ShootEvent
{
    ShootEventType type = ShootEventType::ShootOnce;
};

struct INS_State
{
    float yaw_rad = 0.0f;
    float pitch_rad = 0.0f;
    float roll_rad = 0.0f;
    float gyro_x_rad_s = 0.0f;
    float gyro_y_rad_s = 0.0f;
    float gyro_z_rad_s = 0.0f;
};

/** 两轴目标均为 INS 姿态 rad；速度字段为 IMU 模式的 rad/s 前馈。
 * LOCK 捕获当前姿态并忽略目标字段；自动恢复后 IMU 需发布新目标。
 */
struct GimbalCmd
{
    float yaw_angle_rad = 0.0f;
    float pitch_angle_rad = 0.0f;
    float yaw_speed_rad_s = 0.0f;
    float pitch_speed_rad_s = 0.0f;
    GimbalMode mode = GimbalMode::DISABLED;
};

struct ChassisCmd
{
    float velocity_x_m_s = 0.0f;
    float velocity_y_m_s = 0.0f;
    float angular_velocity_rad_s = 0.0f;
    ChassisMode mode = ChassisMode::ZERO_FORCE;
};

/** 内部物理量采用 SI：发射转速 rad/s，射频 Hz。 */
struct ShootCmd
{
    float friction_speed_rad_s = 0.0f;
    float loader_speed_rad_s = 0.0f;
    float shoot_rate_hz = 0.0f;
    ShootMode shoot_mode = ShootMode::OFF;
    FrictionMode friction_mode = FrictionMode::OFF;
    LoaderMode loader_mode = LoaderMode::STOP;
};

/** 姿态来自 INS；enabled 表示云台功能获许可且两轴均 ready，不表示 CAN 目标已被硬件发送。
 * gyro 轴由云台配置选择；INS 无效时姿态/速度清零，调用者须检查 ins_valid。
 */
struct GimbalFeedback
{
    float yaw_rad = 0.0f;
    float pitch_rad = 0.0f;
    float yaw_speed_rad_s = 0.0f;
    float pitch_speed_rad_s = 0.0f;
    bool ins_valid = false;
    bool enabled = false;
};

struct ChassisFeedback
{
    float velocity_x_m_s = 0.0f;
    float velocity_y_m_s = 0.0f;
    float angular_velocity_rad_s = 0.0f;
    bool enabled = false; ///< 八个 DJI 电机均 ready；ZERO_FORCE 或任一反馈过期时为 false。
    bool online = false;
};

/** DJI 输出轴反馈直接使用 rad 和 rad/s，不经角度制转换。 */
struct ShootFeedback
{
    float friction_left_speed_rad_s = 0.0f;
    float friction_right_speed_rad_s = 0.0f;
    float loader_angle_rad = 0.0f;
    float loader_speed_rad_s = 0.0f;
    bool enabled = false; ///< 摩擦轮和拨弹盘三个 DJI 电机均 ready；OFF 时为 false。
    bool online = false;
};

#endif

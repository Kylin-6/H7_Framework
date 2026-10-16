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

/** UI 状态独立于控制模式，Unknown 不代表停机。 */
enum class Enum_Client_UI_Chassis : uint8_t
{
    Unknown,
    Stopped,
    Waiting,
    Active,
    Spin,
};

enum class Enum_Client_UI_Motion : uint8_t
{
    Unknown,
    Stopped,
    Moving,
};

enum class Enum_Client_UI_AutoAim : uint8_t
{
    Unknown,
    Off,
    Waiting,
    Active,
};

/** ControlTask 唯一发布，StatusTask 消费；原始反馈时间戳不会随采集刷新。
 * 文字为至多 29 字节的 ASCII 加结尾 NUL；报警无需显示时置空。
 * yaw/autoaim/文字由具体机器人适配器提供，整体快照有效期 200 ms。
 */
struct Struct_Client_UI_State
{
    uint64_t pitch_timestamp_us = 0U;
    uint64_t shoot_timestamp_us = 0U;
    uint64_t chassis_timestamp_us = 0U;
    float pitch_rad = 0.0f;
    float relative_yaw_rad = 0.0f;
    float left_speed_rad_s = 0.0f;
    float right_speed_rad_s = 0.0f;
    bool pitch_valid = false;
    bool yaw_valid = false;
    bool speed_valid = false;
    Enum_Client_UI_Chassis chassis = Enum_Client_UI_Chassis::Unknown;
    Enum_Client_UI_Motion friction = Enum_Client_UI_Motion::Unknown;
    Enum_Client_UI_Motion loader = Enum_Client_UI_Motion::Unknown;
    Enum_Client_UI_AutoAim autoaim = Enum_Client_UI_AutoAim::Unknown;
    char mode_text[30] = "UNKNOWN";
    char alert_text[3][30]{};
    bool alert_warning[3]{};
};

#endif

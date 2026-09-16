#ifndef MESSAGE_TYPES_H
#define MESSAGE_TYPES_H

#include <cstdint>

/** 底盘控制模式。 */
enum class ChassisMode : uint8_t
{
    ZERO_FORCE = 0,       ///< 零力矩，电机不主动输出
    NO_FOLLOW,            ///< 底盘运动与云台朝向解耦
    FOLLOW_GIMBAL_YAW,    ///< 底盘跟随云台偏航角
    ROTATE,               ///< 小陀螺旋转模式
};

/** 云台控制模式。 */
enum class GimbalMode : uint8_t
{
    DISABLED = 0, ///< 关闭输出
    IMU,          ///< 使用 INS 姿态闭环
    LOCK,         ///< 保持当前/给定角度
};

/** 发射机构总开关。 */
enum class ShootMode : uint8_t
{
    OFF = 0,
    ON,
};

/** 摩擦轮工作状态。 */
enum class FrictionMode : uint8_t
{
    OFF = 0,
    ON,
};

/** 拨弹盘工作模式。 */
enum class LoaderMode : uint8_t
{
    STOP = 0, ///< 停止
    REVERSE,  ///< 反转退弹
    SINGLE,   ///< 单发
    TRIPLE,   ///< 三连发
    BURST,    ///< 连发
};

/**
 * @brief INS 向应用层发布的最新姿态状态。
 * @note 欧拉角顺序为 yaw-pitch-roll；角速度采用机体坐标系。
 */
struct INS_State
{
    float yaw_rad = 0.0f;       ///< 偏航角，rad
    float pitch_rad = 0.0f;     ///< 俯仰角，rad
    float roll_rad = 0.0f;      ///< 横滚角，rad
    float gyro_x_rad_s = 0.0f;  ///< X 轴角速度，rad/s
    float gyro_y_rad_s = 0.0f;  ///< Y 轴角速度，rad/s
    float gyro_z_rad_s = 0.0f;  ///< Z 轴角速度，rad/s
};

/** RobotCmd 发给云台应用的目标量。 */
struct GimbalCmd
{
    float yaw_angle_rad = 0.0f;       ///< 偏航目标角，rad
    float pitch_angle_rad = 0.0f;     ///< 俯仰目标角，rad
    float yaw_speed_rad_s = 0.0f;     ///< 偏航前馈角速度，rad/s
    float pitch_speed_rad_s = 0.0f;   ///< 俯仰前馈角速度，rad/s
    GimbalMode mode = GimbalMode::DISABLED; ///< 控制模式
};

/** RobotCmd 发给底盘应用的速度目标。 */
struct ChassisCmd
{
    float velocity_x_m_s = 0.0f;          ///< X 方向速度，m/s
    float velocity_y_m_s = 0.0f;          ///< Y 方向速度，m/s
    float angular_velocity_rad_s = 0.0f;  ///< 自转角速度，rad/s
    ChassisMode mode = ChassisMode::ZERO_FORCE; ///< 控制模式
};

/** RobotCmd 发给发射应用的目标量。 */
struct ShootCmd
{
    float friction_speed_deg_s = 0.0f; ///< 摩擦轮目标速度，deg/s
    float loader_speed_deg_s = 0.0f;   ///< 拨弹盘目标速度，deg/s
    float shoot_rate_hz = 0.0f;        ///< 连发频率，Hz
    ShootMode shoot_mode = ShootMode::OFF;          ///< 发射总开关
    FrictionMode friction_mode = FrictionMode::OFF; ///< 摩擦轮状态
    LoaderMode loader_mode = LoaderMode::STOP;      ///< 拨弹模式
};

/** 云台应用反馈给 RobotCmd 的当前状态。 */
struct GimbalFeedback
{
    float yaw_rad = 0.0f;            ///< 当前偏航角，rad
    float pitch_rad = 0.0f;          ///< 当前俯仰角，rad
    float yaw_speed_rad_s = 0.0f;    ///< 当前偏航角速度，rad/s
    float pitch_speed_rad_s = 0.0f;  ///< 当前俯仰角速度，rad/s
    bool ins_valid = false;           ///< INS 数据是否有效
    bool enabled = false;             ///< 云台是否已进入可控状态
};

/** 底盘应用反馈给 RobotCmd 的当前状态。 */
struct ChassisFeedback
{
    float velocity_x_m_s = 0.0f;          ///< 当前 X 方向速度，m/s
    float velocity_y_m_s = 0.0f;          ///< 当前 Y 方向速度，m/s
    float angular_velocity_rad_s = 0.0f;  ///< 当前自转角速度，rad/s
    bool enabled = false;                  ///< 底盘是否使能
    bool online = false;                   ///< 底盘设备是否在线
};

/** 发射应用反馈给 RobotCmd 的当前状态。 */
struct ShootFeedback
{
    float friction_left_speed_deg_s = 0.0f;  ///< 左摩擦轮速度，deg/s
    float friction_right_speed_deg_s = 0.0f; ///< 右摩擦轮速度，deg/s
    float loader_angle_deg = 0.0f;            ///< 拨弹盘角度，deg
    float loader_speed_deg_s = 0.0f;          ///< 拨弹盘速度，deg/s
    bool enabled = false;                      ///< 发射机构是否使能
    bool online = false;                       ///< 发射机构设备是否在线
};

#endif

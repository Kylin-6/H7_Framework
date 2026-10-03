#ifndef GIMBAL_CONFIG_H
#define GIMBAL_CONFIG_H

#include "board_config.h"
#include "fdcan.h"

enum class GimbalGyroAxis : uint8_t
{
    X,
    Y,
    Z
};

struct Struct_Gimbal_Motor_Config
{
    FDCAN_HandleTypeDef* bus = nullptr;
    uint8_t id = 0x03;
    uint16_t feedback_id = 0x005;
    bool reverse = false;
    // 协议量程：rad、rad/s、N·m，须与电机端一致。
    float position_max = 3.14f;
    float velocity_max = 30.0f;
    float torque_max = 10.0f;
};

struct Struct_Gimbal_Config
{
    Struct_Gimbal_Motor_Config yaw;
    Struct_Gimbal_Motor_Config pitch;
    GimbalGyroAxis yaw_gyro_axis = GimbalGyroAxis::Z;
    GimbalGyroAxis pitch_gyro_axis = GimbalGyroAxis::Y;
    float yaw_gyro_sign = 1.0f;
    float pitch_gyro_sign = 1.0f;
    // Yaw 角度环输出 rad/s，速度环输出 N·m。
    float yaw_angle_kp = 8.0f;
    float yaw_speed_limit = 15.0f;
    // 待实机整定：转矩环不是 QD4310 电流环，默认不产生 Yaw 主动转矩。
    float yaw_speed_kp = 0.0f;
    float yaw_speed_ki = 0.0f;
    float yaw_speed_kd = 0.0f;
    float yaw_integral_limit = 0.0f;
    float yaw_torque_limit = 10.0f;
    // Pitch MIT 位置/速度目标及机械限位，单位 rad、rad/s。
    float pitch_kp = 20.0f;
    float pitch_kd = 1.0f;
    float pitch_min = -0.6981317f; // -40°
    float pitch_max = 0.2617994f; // +15°
    float pitch_speed_limit = 1.0f;
    float pitch_motor_per_imu = 1.0f;
};

/**
 * @brief 生成默认机构参数，并从当前 BoardConfig 绑定两轴总线。
 * @return 可供 Gimbal_Init 复制的配置；Pitch ID 单独覆盖为对应参考值。
 * @note 仅构造参数，不注册设备、不访问电机；上板前需核对参数与实际机构。
 */
inline Struct_Gimbal_Config Gimbal_Default_Config()
{
    Struct_Gimbal_Config config;
    config.yaw.bus = BoardConfig_Get().gimbal_yaw_bus;
    config.pitch.bus = BoardConfig_Get().gimbal_pitch_bus;
    config.pitch.id = 0x09;
    config.pitch.feedback_id = 0x019;
    return config;
}

#endif

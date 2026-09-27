#ifndef GIMBAL_CONFIG_H
#define GIMBAL_CONFIG_H

#include "fdcan.h"

enum class GimbalGyroAxis : uint8_t { X, Y, Z };

struct Struct_Gimbal_Motor_Config
{
    FDCAN_HandleTypeDef *bus = nullptr;
    uint8_t id = 1;
    uint16_t feedback_id = 0x101;
    bool reverse = false;
    // Meta 达妙协议示例，必须与电机端 PMAX/VMAX/TMAX 一致，不代表所有型号通用。
    float position_max = 12.5f;
    float velocity_max = 45.0f;
    float torque_max = 18.0f;
};

struct Struct_Gimbal_Config
{
    Struct_Gimbal_Motor_Config yaw;
    Struct_Gimbal_Motor_Config pitch;
    GimbalGyroAxis yaw_gyro_axis = GimbalGyroAxis::Z;
    GimbalGyroAxis pitch_gyro_axis = GimbalGyroAxis::Y;
    float yaw_gyro_sign = 1.0f;
    float pitch_gyro_sign = 1.0f;
    // basic_framework 角度比例增益与 500 deg/s 上限，内部统一弧度。
    float yaw_angle_kp = 8.0f;
    float yaw_speed_limit = 8.72664626f;
    // 待实机整定：转矩环不是 QD4310 电流环，默认不产生 Yaw 主动转矩。
    float yaw_speed_kp = 0.0f;
    float yaw_speed_ki = 0.0f;
    float yaw_speed_kd = 0.0f;
    float yaw_integral_limit = 0.0f;
    float yaw_torque_limit = 18.0f;
    // Meta 小米 Pitch 的 MIT 控制示例，增益及机械限位必须重新核对。
    float pitch_kp = 20.0f;
    float pitch_kd = 1.0f;
    float pitch_min = -1.5f;
    float pitch_max = 0.5f;
    float pitch_speed_limit = 1.0f;
    float pitch_motor_per_imu = 1.0f;
};

inline Struct_Gimbal_Config Gimbal_Default_Config()
{
    Struct_Gimbal_Config config;
    config.yaw.bus = &hfdcan2;
    config.pitch.bus = &hfdcan1;
    config.pitch.id = 2;
    config.pitch.feedback_id = 0x102;
    return config;
}

#endif

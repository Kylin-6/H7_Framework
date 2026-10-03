#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H

#include "../physical_units.h"

#include <cstdint>

// 底盘机构与控制参数；总线归属由 BoardConfig 提供。
struct ChassisPidConfig
{
    float kp;
    float ki;
    float kd;
    float integral_limit;
    float output_limit;
};

struct ChassisConfig
{
    float half_length_m = 0.163f;
    float half_width_m = 0.163f;
    float wheel_radius_m = 0.058f;
    float feedback_alpha = 0.032258f;
    float stop_speed_m_s = 0.001f;
    float steer_offset_rad[4] = {
        DegToRad(102.5f), DegToRad(12.5f),
        DegToRad(137.5f), DegToRad(145.0f)};
    uint8_t motor_id[4] = {1U, 2U, 3U, 4U};
    ChassisPidConfig wheel_speed_pid{4.5f, 0.05f, 0.0f, 3000.0f, 16000.0f};
    ChassisPidConfig steer_angle_pid{30.0f, 0.2f, 0.0f,
                                     DegToRad(200.0f), DegToRad(1000.0f)};
    ChassisPidConfig steer_speed_pid{4.0f, 4.0f, 0.0f, 3000.0f, 15000.0f};
};

constexpr ChassisConfig kChassisConfig{};

#endif

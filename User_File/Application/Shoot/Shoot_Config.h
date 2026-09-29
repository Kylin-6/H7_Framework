#ifndef SHOOT_CONFIG_H
#define SHOOT_CONFIG_H

#include "../physical_units.h"
#include <cstdint>

// 发射机构与控制参数；总线归属由 BoardConfig 提供。
struct ShootPidConfig
{
    float kp;
    float ki;
    float kd;
    float integral_limit;
    float output_limit;
};

struct ShootConfig
{
    float default_friction_speed_rad_s = 25.0f;
    float default_rate_hz = 10.0f;
    float one_bullet_angle_rad = DegToRad(36.0f);
    float reverse_speed_rad_s = DegToRad(-360.0f);
    float friction_gear_ratio = 1.0f; // 摩擦轮直驱，不使用 M3508 默认减速比 19。
    uint8_t friction_left_id = 3U;
    uint8_t friction_right_id = 2U;
    uint8_t loader_id = 8U;
    ShootPidConfig friction_speed_pid{7.5f, 5.0f, 0.0f, 16000.0f, 16000.0f};
    ShootPidConfig loader_current_pid{1.0f, 50.0f, 0.0f, 12000.0f, 12000.0f};
    ShootPidConfig loader_speed_pid{7.5f, 20.0f, 0.0f, 12000.0f, 12000.0f};
    ShootPidConfig loader_angle_pid{10.0f, 0.0f, 0.0f,
                                       0.0f, DegToRad(360.0f)};
};

constexpr ShootConfig kShootConfig{};

#endif

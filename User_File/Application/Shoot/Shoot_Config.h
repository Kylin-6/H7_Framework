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
    float integral_limit; // 积分项输出上限，与该环输出同单位。
    float output_limit; // 角度环为 rad/s；其余环按驱动级联契约配置。
};

struct ShootConfig
{
    float default_friction_speed_rad_s = 25.0f; // 摩擦轮开启且命令速度不为正时的备用输出轴速度。
    float default_rate_hz = 10.0f; // 连发未提供正射速时的备用弹数/s。
    float one_bullet_angle_rad = DegToRad(36.0f); // 拨弹盘输出轴每个弹位的机械角，用于事件和射速换算。
    float reverse_speed_rad_s = DegToRad(-360.0f); // 反转模式未提供非零速度时的备用角速度。
    float friction_gear_ratio = 1.0f; // 摩擦轮直驱，不使用 M3508 默认减速比 19。
    uint8_t friction_left_id = 3U; // 左摩擦轮 DJI 节点编号。
    uint8_t friction_right_id = 2U; // 右摩擦轮 DJI 节点编号，初始化中配置 reverse=true。
    uint8_t loader_id = 8U; // 拨弹电机 DJI 节点编号，必须与总线其他设备兼容。
    ShootPidConfig friction_speed_pid{7.5f, 5.0f, 0.0f, 16000.0f, 16000.0f};
    ShootPidConfig loader_current_pid{1.0f, 50.0f, 0.0f, 12000.0f, 12000.0f};
    ShootPidConfig loader_speed_pid{7.5f, 20.0f, 0.0f, 12000.0f, 12000.0f};
    ShootPidConfig loader_angle_pid{10.0f, 0.0f, 0.0f,
                                    0.0f, DegToRad(360.0f)};
};

constexpr ShootConfig kShootConfig{};

#endif

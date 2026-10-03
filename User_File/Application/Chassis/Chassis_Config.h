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
    float half_length_m = 0.163f; // 几何中心到轮模块的纵向距离，影响旋转速度分量。
    float half_width_m = 0.163f; // 几何中心到轮模块的横向距离，须大于零。
    float wheel_radius_m = 0.058f; // 有效滚动半径，用于 m/s 与输出轴 rad/s 换算。
    float feedback_alpha = 0.032258f; // 每个 1 ms 周期的一阶平滑权重，不是 100 Hz 发布周期的权重。
    float stop_speed_m_s = 0.001f; // 近零速度阈值，低于它时行走轮停转并保持当前舵角。
    // 轮索引与运动学数组一致；这是机械偏置，不能替代增量编码器的上电寻零。
    float steer_offset_rad[4] = {
        DegToRad(102.5f), DegToRad(12.5f),
        DegToRad(137.5f), DegToRad(145.0f)};
    uint8_t motor_id[4] = {1U, 2U, 3U, 4U}; // 电机编号而非 CAN 报文 ID；两组在各自总线上使用相同索引。
    ChassisPidConfig wheel_speed_pid{4.5f, 0.05f, 0.0f, 3000.0f, 16000.0f};
    ChassisPidConfig steer_angle_pid{30.0f, 0.2f, 0.0f,
                                     DegToRad(200.0f), DegToRad(1000.0f)};
    ChassisPidConfig steer_speed_pid{4.0f, 4.0f, 0.0f, 3000.0f, 15000.0f};
};

constexpr ChassisConfig kChassisConfig{};

#endif

#ifndef __GIMBAL_H
#define __GIMBAL_H

#include "message_types.h"

/** ControlTask 的 1 kHz 入口；状态发布为 100 Hz。 */
void Gimbal_Update(void);

#if GIMBAL
#include "Gimbal_Config.h"
#include "alg_pid.h"
#include "dmmotor.h"

enum Enum_Gimbal_Status
{
    Gimbal_Status_DISABLE,
    Gimbal_Status_ENABLING,
    Gimbal_Status_READY,
    Gimbal_Status_FAULT,
    Gimbal_Status_CONFIG_ERROR,
};

struct Struct_Gimbal
{
    Class_DMMotor Yaw_Motor;
    Class_DMMotor Pitch_Motor;
    Class_PID Yaw_Angle_PID;
    Class_PID Yaw_Speed_PID;
    Enum_Gimbal_Status status = Gimbal_Status_DISABLE;
    float Target_Yaw_Angle = 0.0f;
    float Target_Pitch_Angle = 0.0f;
    float Target_Yaw_Speed = 0.0f;
    float Target_Pitch_Speed = 0.0f;
};

extern Struct_Gimbal Gimbal;
/** 仅启动阶段调用一次；复制配置并注册驱动，不等待应答、不置零、不切换电机模式。 */
bool Gimbal_Init(const Struct_Gimbal_Config &config = Gimbal_Default_Config());
#endif
#endif

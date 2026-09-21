#ifndef __GIMBAL_H
#define __GIMBAL_H

#include "message_types.h"

/** 云台应用的 1 kHz 周期入口，由 Control_Task 调度。 */
void Gimbal_Update(void);

#if GIMBAL

#include "QD4310.h"
#include "alg_pid.h"
#include "alg_fsm.h"
#include "bsp_can.h"

#define YAW_ID 0
#define PITCH_ID 1
#define GIMBAL_PITCH_MIN_ANGLE_RAD 5.85f
#define GIMBAL_PITCH_MAX_ANGLE_RAD 6.25f

enum Enum_Gimbal_Status
{
    Gimbal_Status_DISABLE = 0, ///< 尚未初始化或已禁用
    Gimbal_Status_READY,       ///< 两轴电机均已就绪
    Gimbal_Status_YAW_ERROR,   ///< Yaw 轴使能或通信异常
    Gimbal_Status_PITCH_ERROR, ///< Pitch 轴使能或通信异常
};


typedef struct
{
    Class_FSM<5> Gimbal_FSM; ///< 云台初始化状态机
    QD4310_t Yaw_Motor;      ///< 偏航轴电机
    QD4310_t Pitch_Motor;    ///< 俯仰轴电机

    float Target_Yaw_Angle;   ///< Yaw 目标角，rad
    float Target_Pitch_Angle; ///< Pitch 目标角，rad
    float Target_Yaw_Speed;   ///< Yaw 目标角速度，rad/s
    float Target_Pitch_Speed; ///< Pitch 目标角速度，rad/s

    Class_PID Yaw_Angle_PID;   ///< Yaw 角度外环
    Class_PID Pitch_Angle_PID; ///< Pitch 角度外环，当前预留
    Class_PID Yaw_Speed_PID;   ///< Yaw 速度内环
    Class_PID Pitch_Speed_PID; ///< Pitch 速度环

}QDGimbal_t;


/** 初始化云台设备和控制器；电机异常时在有限重试后返回。 */
void Gimbal_Init(void);
/** 执行一次已经通过状态守卫的云台闭环计算。 */
void Gimbal_Loop(void);

extern QDGimbal_t Gimbal;

#endif
#endif

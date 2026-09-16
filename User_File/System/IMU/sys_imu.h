/**
 * @file sys_imu.h
 * @author zzm
 * @brief IMU系统级配置与状态发布入口
 * @version 1.0
 * @date 2026-08-12
 *
 * @details
 * 本模块集中设置整机使用的IMU参数，并将已完成的姿态解算结果转换为系统消息。
 * 传感器初始化、数据采集和姿态解算仍由Class_BMI088和VQF算法类完整实现。
 */

#ifndef __SYS_IMU_H
#define __SYS_IMU_H

/* Includes ------------------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 将整机选定的VQF参数写入BMI088
 *
 * @details
 * 必须在Class_BMI088::Init()之前调用。该函数只写入配置，便于在一个位置调整
 * 采样周期、姿态修正速度、零偏估计和静止判定策略。
 */
void System_IMU_Configure();

/**
 * @brief 将最新姿态解算结果发布为INS_State
 * @note 仅在任务上下文、完成一批姿态解算后调用。
 */
void System_IMU_Publish_State();

#endif

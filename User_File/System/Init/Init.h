/**
 * @file Init.h
 * @brief 系统初始化入口、总体状态和失败位图。
 * @details
 * 初始化结果将“是否允许控制任务运行”和“哪些可选设备不可用”分开表达：
 * FATAL 阻止控制任务启动，DEGRADED 允许其余模块继续运行。调用方应同时
 * 读取状态和失败位图，不能只用 init_finished 判断所有设备均已就绪。
 */
#ifndef __INIT_H
#define __INIT_H

#include <stdint.h>

typedef enum
{
    SYSTEM_INIT_READY = 0, ///< 必需和可选模块均初始化成功。
    SYSTEM_INIT_DEGRADED,  ///< 可选模块失败，系统继续以降级能力运行。
    SYSTEM_INIT_FATAL,     ///< 控制时基等必需资源失败，控制任务不得运行。
} Enum_System_Init_State;

/** @brief 初始化失败来源位图；多个失败位可以同时置位。 */
typedef enum
{
    SYSTEM_INIT_FAILURE_NONE = 0U,
    SYSTEM_INIT_FAILURE_TIM4 = 1U << 0,
    SYSTEM_INIT_FAILURE_TIM5 = 1U << 1,
    SYSTEM_INIT_FAILURE_BMI088 = 1U << 2,
    SYSTEM_INIT_FAILURE_W25Q64 = 1U << 3,
    SYSTEM_INIT_FAILURE_ADC1 = 1U << 4,
} Enum_System_Init_Failure;



#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 按依赖顺序初始化 BSP 与板载设备，并记录失败而不是无限等待。
 * @note 函数返回只表示初始化流程结束；实际结果由 System_Init_GetState() 判断。
 */
void System_Init(void);

/** @brief 获取本次 System_Init() 汇总出的最高严重级别。 */
Enum_System_Init_State System_Init_GetState(void);

/** @brief 获取 Enum_System_Init_Failure 各位组成的失败快照。 */
uint32_t System_Init_GetFailureMask(void);

#ifdef __cplusplus
}
#endif

#endif /* __INIT_H */

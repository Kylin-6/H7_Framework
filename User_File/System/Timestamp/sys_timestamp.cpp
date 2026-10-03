/**
 * @file sys_timestamp.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief 时间计算相关支持库
 * @version 0.1
 * @date 2025-08-16 0.1 新建文档
 *
 * @copyright USTC-RoboWalker (c) 2025
 *
 */

/**
 * 要求：绑定 32 位定时器，计数频率 1 MHz，ARR = 3600000000 - 1。
 * 当前使用 TIM5，每计数 1 us，更新中断每 3600 s 扩展一次溢出计数。
 * Init 只绑定句柄；启动定时器与转发更新中断由 System_Init/callback 负责。
 */

/* Includes ------------------------------------------------------------------*/

#include "sys_timestamp.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

Class_Timestamp SYS_Timestamp;

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化时间戳
 *
 * @param htim 绑定的 32 位定时器；计数频率 1 MHz，ARR = 3599999999。
 */
void Class_Timestamp::Init(TIM_HandleTypeDef *htim)
{
    TIM_Handler = htim;
}

/**
 * @brief TIM定时器回调函数, 每3600s调用一次
 *
 */
void Class_Timestamp::TIM_3600s_PeriodElapsedCallback()
{
    TIM_Overflow_Count++;
}

/**
 * @brief 计算当前时间戳, 单位微秒
 *
 * @return uint64_t 当前时间戳
 */
uint64_t Class_Timestamp::Calculate_Timestamp() const
{
    for (;;)
    {
        // 前后两次读取溢出次数，排除 CNT 读取期间更新中断已经执行的情况。
        const uint32_t overflow_before = TIM_Overflow_Count;
        const uint32_t counter = TIM_Handler->Instance->CNT;
        __DMB();
        const uint32_t overflow_after = TIM_Overflow_Count;
        if (overflow_before != overflow_after)
        {
            continue;
        }

        // 硬件已回绕但 ISR 尚未更新软件计数时，临时补算一次周期而不修改共享状态。
        if (__HAL_TIM_GET_FLAG(TIM_Handler, TIM_FLAG_UPDATE) != RESET)
        {
            const uint32_t pending_counter = TIM_Handler->Instance->CNT;
            __DMB();
            if (overflow_before != TIM_Overflow_Count)
            {
                continue;
            }
            return (static_cast<uint64_t>(overflow_before) + 1ULL) * TIM_PERIOD_US +
                   static_cast<uint64_t>(pending_counter);
        }

        return static_cast<uint64_t>(overflow_before) * TIM_PERIOD_US +
               static_cast<uint64_t>(counter);
    }
}

/**
 * @brief 延迟指定秒数
 *
 * @param Second 延迟秒数
 */
void Namespace_SYS_Timestamp::Delay_Second(const uint32_t &Second)
{
    volatile uint64_t start_time = SYS_Timestamp.Get_Current_Timestamp();

    while ((uint64_t)(Second) * 1000000ULL + start_time > SYS_Timestamp.Get_Current_Timestamp())
    {
    }
}

/**
 * @brief 延迟指定毫秒数
 *
 * @param Millisecond 延迟毫秒数
 */
void Namespace_SYS_Timestamp::Delay_Millisecond(const uint32_t &Millisecond)
{
    volatile uint64_t start_time = SYS_Timestamp.Get_Current_Timestamp();

    while ((uint64_t)(Millisecond) * 1000ULL + start_time > SYS_Timestamp.Get_Current_Timestamp())
    {
    }
}

/**
 * @brief 延迟指定微秒数
 *
 * @param Microsecond 延迟微秒数
 */
void Namespace_SYS_Timestamp::Delay_Microsecond(const uint32_t &Microsecond)
{
    volatile uint64_t start_time = SYS_Timestamp.Get_Current_Timestamp();

    while ((uint64_t)(Microsecond) + start_time > SYS_Timestamp.Get_Current_Timestamp())
    {
    }
}

/**
 * @brief  C 语言可用的时间戳获取函数 (微秒)
 * @return uint64_t 当前时间戳, 单位微秒
 * @note   纯 C 包装, 内部调用 SYS_Timestamp.Get_Now_Microsecond()
 */
extern "C" uint64_t SYS_Timestamp_Get_Microsecond(void)
{
    return SYS_Timestamp.Get_Now_Microsecond();
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/

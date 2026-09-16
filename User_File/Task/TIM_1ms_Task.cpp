/**
 * @file TIM_1ms_Task.cpp
 * @author zzm
 * @brief 定时器回调分发器, 按周期调度各模块的周期处理函数
 * @version 1.0
 * @date 2026-06-06 1.0 接入 W25Q64JV 自动轮询超时检测
 *
 * @details
 * 通过编译期静态回调表分发不同周期的处理函数:
 *   - 1ms:  W25Q64JV 超时检测 / 按键扫描
 *   - 10ms: WS2812 灯效刷新
 *   - 50ms: 按键消抖
 *   - 128ms: BMI088 姿态解算
 *
 * @copyright USTC-RoboWalker (c) 2026
 */

#include "user_task.h"
#include "bsp_key.h"
#include "bsp_uart.h"
#include "bsp_w25q64jv.h"
#include <cstddef>

/**
 * @brief W25Q64JV 自动轮询超时检测回调 (1ms)
 *
 * @note  检测 OSPI 硬件自动轮询是否超时, 防止 Busy_Flag 永久锁死
 * @note  extern "C" 链路: TIM1msTask → Pulse_Dispatch → 本函数 → BSP_W25Q64JV.TIM_1ms_AutoPollingTimeout_PeriodElapsedCallback()
 */
extern "C" {
void W25Q64JV_AutoPolling_Callback(void) {
    BSP_W25Q64JV.TIM_1ms_AutoPollingTimeout_PeriodElapsedCallback();
}
}

static const PulseEntry_t TIM_1ms_Callback_Table[] = {
    {1U, W25Q64JV_AutoPolling_Callback},
    {1U, BSP_Key_TIM_1ms_Process_PeriodElapsedCallback},
    {1U, BMI088_TIM_1ms_Service_PeriodElapsedCallback},
    {1U, UART_TIM_1ms_Recover_PeriodElapsedCallback},
    {10U, BSP_WS2812_TIM_10ms_Write_PeriodElapsedCallback},
    {50U, BSP_Key_TIM_50ms_Process_PeriodElapsedCallback},
    {128U, BMI088_TIM_128ms_Calculate_PeriodElapsedCallback},
};


/**
 * @brief 1ms 定时器任务入口 (RTOS 线程)
 *
 * @note  每 1ms 执行一次，通过静态表分发各周期回调
 * @note  使用 osDelayUntil 绝对延时保证严格1 ms周期，不受回调表执行耗时影响
 */
extern "C" void TIM1msTask(void *argument) {
  (void)argument;
  uint32_t xLastWakeTime = osKernelGetTickCount();
  uint32_t pulse_tick_ms = 0U;
  while (1) {
      Pulse_Dispatch(TIM_1ms_Callback_Table,
                     sizeof(TIM_1ms_Callback_Table) / sizeof(TIM_1ms_Callback_Table[0]),
                     pulse_tick_ms);
      pulse_tick_ms++;

      xLastWakeTime += 1;
      osDelayUntil(xLastWakeTime);
  }
}

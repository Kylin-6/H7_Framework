/**
 * @file    CanTxTask.cpp
 * @brief   High 优先级、1 ms CAN 软件发送任务。
 * @details osDelayUntil 阻塞等待；每条 FDCAN 最多尝试一帧离散 FIFO，随后处理
 *          latest-value 周期槽。输入是 CAN BSP 软件通道，输出是 HAL Tx FIFO 提交；
 *          不负责业务编解码、对端确认或电机执行。
 * @author  zzm
 * @version 1.0
 * @date    2026-05-16
 */

/* Includes ------------------------------------------------------------------*/

#include "user_task.h"

#include "bsp_can.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

extern "C" void Can_Tx_Task(void *argument)
{
  uint32_t xLastWakeTime = osKernelGetTickCount();
  for (;;) {
    xLastWakeTime += 1;
    osDelayUntil(xLastWakeTime);
    BSP_CAN_SendAsync();
    BSP_CAN_SendPer();
  }
}

/**
 * @file    TransportTask.cpp
 * @brief   Normal 优先级、约 1 ms 的 USB CDC 调试遥测任务。
 * @details 初始化 USB，周期调用 EricTool 输出；osDelay(1) 允许阻塞。
 *          不负责 System/Transport 的板间 CAN，也不解析遥控/裁判协议。
 * @todo    文件名与板间 Transport 易混淆，后续独立更名，本次保留任务符号。
 * @author  zzm
 * @version 1.2
 * @date    2026-07-11 1.2 移除未使用的 PID tuner
 */

/* Includes ------------------------------------------------------------------*/

#include "sys_debug.h"
#include "usb_device.h"
#include "user_task.h"


/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

extern "C" void Transport_Task(void *argument)
{
    MX_USB_DEVICE_Init();
    EricTool_USB.Set_Data(3, (int) &Debug_IMU_Data.Euler_Yaw_rad,
                         (int) &Debug_IMU_Data.Euler_Pitch_rad,
                         (int) &Debug_IMU_Data.Euler_Roll_rad);
    for (;;)
    {
        EricTool_USB.TIM_1ms_Write_PeriodElapsedCallback();
        osDelay(1);
    }
}

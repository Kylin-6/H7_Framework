/**
 * @file StatusTask.cpp
 * @brief 低频设备在线状态检查任务。
 * @details 调度 DaemonManager，并在装有 DM 电机的固件上限频服务入队失败的恢复命令。
 */

#include "daemon.h"
#include "cmsis_os2.h"
#if H7_HAS_DM_MOTOR
#include "dmmotor.h"
#endif

extern "C" void Status_Task(void *argument)
{
    (void)argument;
    // 使用绝对唤醒时间，避免 CheckAll() 执行时间累积到任务周期中。
    uint32_t next_wake_tick = osKernelGetTickCount();

    for (;;)
    {
        // 100 Hz 足以覆盖当前最短 100 ms 设备超时，并远低于 1 kHz 控制频率。
        DaemonManager::CheckAll();
#if H7_HAS_DM_MOTOR
        Class_DMMotor::ServiceAll();
#endif
        next_wake_tick += 10U;
        osDelayUntil(next_wake_tick);
    }
}

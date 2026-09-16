/**
 * @file StatusTask.cpp
 * @brief 低频设备在线状态检查任务。
 * @details 本任务只调度 DaemonManager，不执行安全策略、日志或设备控制。
 */

#include "daemon.h"
#include "cmsis_os2.h"

extern "C" void Status_Task(void *argument)
{
    (void)argument;
    // 使用绝对唤醒时间，避免 CheckAll() 执行时间累积到任务周期中。
    uint32_t next_wake_tick = osKernelGetTickCount();

    for (;;)
    {
        // 100 Hz 足以覆盖当前最短 100 ms 设备超时，并远低于 1 kHz 控制频率。
        DaemonManager::CheckAll();
        next_wake_tick += 10U;
        osDelayUntil(next_wake_tick);
    }
}

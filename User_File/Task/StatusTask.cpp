/**
 * @file StatusTask.cpp
 * @brief Low-frequency device online-state inspection.
 */

#include "daemon.h"
#include "cmsis_os2.h"

extern "C" void Status_Task(void *argument)
{
    (void)argument;
    uint32_t next_wake_tick = osKernelGetTickCount();

    for (;;)
    {
        DaemonManager::CheckAll();
        next_wake_tick += 10U;
        osDelayUntil(next_wake_tick);
    }
}

/**
 * @file StatusTask.cpp
 * @brief 低频设备在线状态检查任务。
 * @details Low 优先级、100 Hz，osDelayUntil 阻塞等待；输入是设备 Feed 时间戳和
 *          DM 待恢复标记，输出是在线跃迁检查及入队失败的限频重试。不解析 CAN，
 *          不负责整车安全策略或电机控制算法。
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

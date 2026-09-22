/**
 * @file Control_Task.cpp
 * @brief 应用层统一控制任务。
 * @details
 * Task 只提供 1 kHz 调度，不承载具体控制算法。RobotCmd 先更新命令，随后依次
 * 调度云台、底盘和发射 Application，各模块直接控制自己拥有的 Device。
 */

#include "Chassis.h"
#include "Gimbal.h"
#include "RobotCmd.h"
#include "Shoot.h"
#include "Init.h"
#include "user_task.h"

extern "C" void Control_Task(void* argument)
{
    // 在每个 1 kHz CAN 发送周期前生成最新目标；BMI088 High2 任务仍优先处理传感器数据。
    osThreadSetPriority(osThreadGetId(), osPriorityHigh1);

    if (System_Init_GetState() == SYSTEM_INIT_FATAL)
    {
        for (;;)
        {
            osDelay(1000U);
        }
    }

#if GIMBAL
    Gimbal_Init();
#endif
    (void)Chassis_Init();
    (void)Shoot_Init();
    RobotCmd_Init();
    // Balance_init();

    for (;;)
    {
        /* 由 1 ms 定时回调唤醒；阻塞等待期间不占用 CPU。 */
        osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);
        /* 命令所有者先发布最新目标，再由各 Application 消费并执行。 */
        RobotCmd_Update();
        Gimbal_Update();
        Chassis_Update();
        Shoot_Update();
        // Balance_loop();
    }
}

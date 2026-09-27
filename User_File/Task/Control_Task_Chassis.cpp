/**
 * @file Control_Task_Chassis.cpp
 * @brief ChassisBoard 的 High1、1 kHz 控制任务。
 * @details 阻塞等待线程标志；先将板间命令发布到本地 Topic，再更新 Chassis 并
 *          产生本地反馈。周期内不等待 CAN 发送；不拥有 RobotCmd 或动态路由。
 */
#include "Chassis.h"
#include "Init.h"
#include "board_transport.h"
#include "cmsis_os2.h"

extern "C" void Control_Task(void *)
{
    osThreadSetPriority(osThreadGetId(), osPriorityHigh1);
    if (System_Init_GetState() == SYSTEM_INIT_FATAL)
    {
        for (;;) osDelay(1000U);
    }

    BoardTransport_Init();
    (void)Chassis_Init();
    for (;;)
    {
        osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);
        BoardTransport_Poll();
        Chassis_Update();
    }
}

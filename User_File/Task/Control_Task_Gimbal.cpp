/**
 * @file Control_Task_Gimbal.cpp
 * @brief GimbalBoard 的 High1、1 kHz 控制任务。
 * @details 阻塞等待线程标志；先处理板间反馈，再由 RobotCmd 输出本地云台/发射和
 *          远端底盘命令，最后更新 Gimbal/Shoot。周期内不等待 CAN/电机执行，
 *          不负责遥控/视觉解析或动态路由。
 */
#include "Gimbal.h"
#include "RobotCmd.h"
#include "Shoot.h"
#include "Init.h"
#include "board_transport.h"
#include "message_center.h"
#include "output.h"
#include "remote_publisher.h"
#include "cmsis_os2.h"

extern "C" void Control_Task(void *)
{
    osThreadSetPriority(osThreadGetId(), osPriorityHigh1);
    if (System_Init_GetState() == SYSTEM_INIT_FATAL)
    {
        for (;;) osDelay(1000U);
    }

    BoardTransport_Init();
    static LocalPublisher<GimbalCmd> gimbal_output(MessageCenter::Gimbal_Command_Topic);
    static RemotePublisher<ChassisCmd> chassis_output;
    static LocalPublisher<ShootCmd> shoot_output(MessageCenter::Shoot_Command_Topic);
    if (!RobotCmd_Init(gimbal_output.Bind(), chassis_output.Bind(), shoot_output.Bind()))
    {
        for (;;) osDelay(1000U);
    }
    Gimbal_Init();
    (void)Shoot_Init();

    for (;;)
    {
        osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);
        BoardTransport_Poll();
        RobotCmd_Update();
        Gimbal_Update();
        Shoot_Update();
    }
}

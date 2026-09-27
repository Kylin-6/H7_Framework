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

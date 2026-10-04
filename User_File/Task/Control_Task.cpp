/**
 * @file Control_Task.cpp
 * @brief 应用层统一控制任务。
 * @details
 * SingleBoard：High1，阻塞等待 1 ms 线程标志；输入为本地 Topic/应用命令，
 * 输出为各应用所属设备目标和反馈。RobotCmd 先发布，再运行 Gimbal/Chassis/Shoot。
 * 周期内不得等待 I/O；本任务不解析协议、不承担板间 CAN 传输。
 */

#if CHASSIS
#include "Chassis.h"
#endif
#if GIMBAL
#include "Gimbal.h"
#endif
#include "RobotCmd.h"
#include "remote_input.h"
#if SHOOT
#include "Shoot.h"
#endif
#include "Init.h"
#include "message_center.h"
#include "output.h"
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

    static LocalPublisher<GimbalCmd> gimbal_output(MessageCenter::Gimbal_Command_Topic);
    static LocalPublisher<ChassisCmd> chassis_output(MessageCenter::Chassis_Command_Topic);
    static LocalPublisher<ShootCmd> shoot_output(MessageCenter::Shoot_Command_Topic);
    if (!RobotCmd_Init(gimbal_output.Bind(), chassis_output.Bind(), shoot_output.Bind(), SHOOT != 0))
    {
        for (;;) osDelay(1000U);
    }
    (void)RemoteInput_Init();
#if GIMBAL
    Gimbal_Init();
#endif
#if CHASSIS
    (void)Chassis_Init();
#endif
#if SHOOT
    (void)Shoot_Init();
#endif
    // Balance_init();

    for (;;)
    {
        /* 由 1 ms 定时回调唤醒；阻塞等待期间不占用 CPU。 */
        osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);
        RemoteInput_Update();
        /* 命令所有者先发布最新目标，再由各 Application 消费并执行。 */
        RobotCmd_Update();
#if GIMBAL
        Gimbal_Update();
#endif
#if CHASSIS
        Chassis_Update();
#endif
#if SHOOT
        Shoot_Update();
#endif
        // Balance_loop();
    }
}

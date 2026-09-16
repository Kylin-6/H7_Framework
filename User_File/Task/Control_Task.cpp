#include "Chassis.h"
#include "Gimbal.h"
#include "RobotCmd.h"
#include "Shoot.h"
#include "user_task.h"

extern "C" void Control_Task(void* argument)
{
    // 在每个 1 kHz CAN 发送周期前生成最新目标；BMI088 High2 任务仍优先处理传感器数据。
    osThreadSetPriority(osThreadGetId(), osPriorityHigh1);

#if GIMBAL
    Gimbal_Init();
#endif
    (void)Chassis_Init();
    (void)Shoot_Init();
    RobotCmd_Init();
    // Balance_init();

    for (;;)
    {
        osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);
        RobotCmd_Update();
        Gimbal_Update();
        Chassis_Update();
        Shoot_Update();
        // Balance_loop();
    }
}

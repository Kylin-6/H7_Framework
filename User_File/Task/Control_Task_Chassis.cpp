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

#include "user_task.h"

extern "C" void Ins_Task(void* argument)
{
    (void)argument;

    // 预留任务：当前姿态解算由BMI088_Task完成，不占用线程栈和调度时间。
    osThreadExit();
}

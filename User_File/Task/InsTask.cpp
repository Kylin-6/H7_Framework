#include "user_task.h"

extern "C" void Ins_Task(void* argument)
{
    (void)argument;

    // 单板仍会创建此兼容任务并分配栈；退出后不再周期调度，姿态解算由 BMI088_Task 完成。
    osThreadExit();
}

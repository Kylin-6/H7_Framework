#include "board_tasks.h"

void Board_CreateTasks(void)
{
    TransportTaskHandle = osThreadNew(Transport_Task, NULL, &TransportTask_attributes);
    InsTaskHandle = osThreadNew(Ins_Task, NULL, &InsTask_attributes);
    CanTxTaskHandle = osThreadNew(Can_Tx_Task, NULL, &CanTxTask_attributes);
    StatusTaskHandle = osThreadNew(Status_Task, NULL, &StatusTask_attributes);
    TIM_1ms_TaskHandle = osThreadNew(TIM1msTask, NULL, &TIM_1ms_Task_attributes);
    BMI088TaskHandle = osThreadNew(BMI088_Task, NULL, &BMI088Task_attributes);
    ControlTaskHandle = osThreadNew(Control_Task, NULL, &ControlTask_attributes);
    StorageTaskHandle = osThreadNew(Storage_Task, NULL, &StorageTask_attributes);
}

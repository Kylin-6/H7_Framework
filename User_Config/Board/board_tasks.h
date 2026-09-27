#ifndef H7_BOARD_TASKS_H
#define H7_BOARD_TASKS_H

#include "cmsis_os2.h"

#ifdef __cplusplus
extern "C" {
#endif

extern osThreadId_t TransportTaskHandle, InsTaskHandle, CanTxTaskHandle;
extern osThreadId_t StatusTaskHandle, TIM_1ms_TaskHandle, BMI088TaskHandle;
extern osThreadId_t ControlTaskHandle, StorageTaskHandle;
extern const osThreadAttr_t TransportTask_attributes, InsTask_attributes;
extern const osThreadAttr_t CanTxTask_attributes, StatusTask_attributes;
extern const osThreadAttr_t TIM_1ms_Task_attributes, BMI088Task_attributes;
extern const osThreadAttr_t ControlTask_attributes, StorageTask_attributes;

void Transport_Task(void *);
void Ins_Task(void *);
void Can_Tx_Task(void *);
void Status_Task(void *);
void TIM1msTask(void *);
void BMI088_Task(void *);
void Control_Task(void *);
void Storage_Task(void *);
void Board_CreateTasks(void);

#ifdef __cplusplus
}
#endif
#endif

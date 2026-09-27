#include "board_tasks.h"

#include "FreeRTOS.h"
#include "task.h"

osThreadId_t TransportTaskHandle;
osThreadId_t InsTaskHandle;
osThreadId_t CanTxTaskHandle;
osThreadId_t StatusTaskHandle;
osThreadId_t TIM_1ms_TaskHandle;
osThreadId_t BMI088TaskHandle;
osThreadId_t ControlTaskHandle;
osThreadId_t StorageTaskHandle;

const osThreadAttr_t TransportTask_attributes = {
    .name = "TransportTask", .stack_size = 2048 * 4,
    .priority = osPriorityNormal,
};
const osThreadAttr_t InsTask_attributes = {
    .name = "InsTask", .stack_size = 2048 * 4,
    .priority = osPriorityHigh1,
};
const osThreadAttr_t CanTxTask_attributes = {
    .name = "CanTxTask", .stack_size = 1024 * 4,
    .priority = osPriorityHigh,
};
static StaticTask_t StatusTaskControlBlock;
static StackType_t StatusTaskBuffer[512];
const osThreadAttr_t StatusTask_attributes = {
    .name = "StatusTask",
    .cb_mem = &StatusTaskControlBlock,
    .cb_size = sizeof(StatusTaskControlBlock),
    .stack_mem = StatusTaskBuffer,
    .stack_size = sizeof(StatusTaskBuffer),
    .priority = osPriorityLow,
};
const osThreadAttr_t TIM_1ms_Task_attributes = {
    .name = "TIM_1ms_Task", .stack_size = 1024 * 4,
    .priority = osPriorityLow,
};
const osThreadAttr_t BMI088Task_attributes = {
    .name = "BMI088Task", .stack_size = 2048 * 4,
    .priority = osPriorityLow,
};
const osThreadAttr_t ControlTask_attributes = {
    .name = "ControlTask", .stack_size = 2048 * 4,
    .priority = osPriorityLow,
};
const osThreadAttr_t StorageTask_attributes = {
    .name = "StorageTask", .stack_size = 1024 * 4,
    .priority = osPriorityLow,
};

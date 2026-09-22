

#ifndef __INIT_H
#define __INIT_H

#include <stdint.h>

typedef enum
{
    SYSTEM_INIT_READY = 0,
    SYSTEM_INIT_DEGRADED,
    SYSTEM_INIT_FATAL,
} Enum_System_Init_State;

typedef enum
{
    SYSTEM_INIT_FAILURE_NONE = 0U,
    SYSTEM_INIT_FAILURE_TIM4 = 1U << 0,
    SYSTEM_INIT_FAILURE_TIM5 = 1U << 1,
    SYSTEM_INIT_FAILURE_BMI088 = 1U << 2,
    SYSTEM_INIT_FAILURE_W25Q64 = 1U << 3,
    SYSTEM_INIT_FAILURE_ADC1 = 1U << 4,
} Enum_System_Init_Failure;



#ifdef __cplusplus
extern "C" {
#endif

void System_Init(void);
Enum_System_Init_State System_Init_GetState(void);
uint32_t System_Init_GetFailureMask(void);

#ifdef __cplusplus
}
#endif

#endif /* __INIT_H */

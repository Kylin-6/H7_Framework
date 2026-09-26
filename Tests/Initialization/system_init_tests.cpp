#include "Init.h"
#include "system_devices.h"
#include <stdio.h>
#include <stdlib.h>

bool test_imu_ready, test_flash_ready, test_adc_ready;
int test_tim4_status, test_tim5_status;
unsigned test_fifo_starts, test_other_inits;
TestIMU BSP_BMI088;
TestFlash BSP_W25Q64JV;
TestDevice BSP_WS2812, BSP_Buzzer, BSP_Key, BSP_Power, EricTool_USB;
TestClock SYS_Timestamp;
int htim4, htim5, hspi2, hspi6, hospi2, hadc1;
int huart1, huart2, huart3, huart4, huart5, huart6, huart7, huart8, huart9, huart10;
extern volatile bool init_finished;

int main()
{
    // Exercise all failure combinations, ending healthy to verify state reset.
    for (unsigned failures = 31; ; --failures)
    {
        test_tim4_status = (failures & SYSTEM_INIT_FAILURE_TIM4) ? HAL_ERROR : HAL_OK;
        test_tim5_status = (failures & SYSTEM_INIT_FAILURE_TIM5) ? HAL_ERROR : HAL_OK;
        test_imu_ready = !(failures & SYSTEM_INIT_FAILURE_BMI088);
        test_flash_ready = !(failures & SYSTEM_INIT_FAILURE_W25Q64);
        test_adc_ready = !(failures & SYSTEM_INIT_FAILURE_ADC1);
        test_fifo_starts = test_other_inits = 0;
        const unsigned fatal = failures & (SYSTEM_INIT_FAILURE_TIM4 | SYSTEM_INIT_FAILURE_TIM5);
        const unsigned expected_mask = fatal ? fatal : failures;
        const auto expected_state = fatal ? SYSTEM_INIT_FATAL :
            (failures ? SYSTEM_INIT_DEGRADED : SYSTEM_INIT_READY);
        System_Init();
        if (!init_finished || System_Init_GetFailureMask() != expected_mask ||
            System_Init_GetState() != expected_state ||
            test_fifo_starts != (unsigned)(!fatal && test_imu_ready) ||
            test_other_inits != (fatal ? 0U : 5U))
        {
            fprintf(stderr, "FAIL startup with injected mask %u\n", failures);
            return 1;
        }
        printf("PASS startup with injected mask %u\n", failures);
        if (failures == 0) break;
    }
    return 0;
}

#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
enum HAL_StatusTypeDef { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT };
enum GPIO_PinState { GPIO_PIN_RESET, GPIO_PIN_SET };
struct GPIO_TypeDef {};
struct GPIO_InitTypeDef { uint32_t Pin, Mode, Pull; };
struct TIM_HandleTypeDef {};
struct DMA_HandleTypeDef { unsigned ErrorCode, State, Lock; };
struct SPI_HandleTypeDef {
    DMA_HandleTypeDef *hdmatx, *hdmarx;
    unsigned ErrorCode, State, Lock, TxXferCount, RxXferCount;
};
enum { HAL_DMA_ERROR_NONE = 0, HAL_DMA_STATE_READY = 0, HAL_UNLOCKED = 0,
       HAL_SPI_ERROR_NONE = 0, HAL_SPI_STATE_READY = 0 };
extern GPIO_TypeDef test_gpio;
extern TIM_HandleTypeDef htim3;
extern uint32_t test_pwm;
extern HAL_StatusTypeDef test_pwm_status;
#define BMI088_ACCEL__SPI_CS_GPIO_Port (&test_gpio)
#define BMI088_GYRO__SPI_CS_GPIO_Port (&test_gpio)
#define BMI088_GYRO__INTERRUPT_GPIO_Port (&test_gpio)
#define BMI088_ACCEL__SPI_CS_Pin 1
#define BMI088_GYRO__SPI_CS_Pin 2
#define BMI088_GYRO__INTERRUPT_Pin 4
#define BMI088_ACCEL__INTERRUPT_Pin 8
#define GPIO_MODE_IT_RISING 1
#define GPIO_NOPULL 0
#define TIM_CHANNEL_4 4
#define __HAL_TIM_SET_COMPARE(timer, channel, value) (test_pwm = (value))
inline HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *, uint32_t) { return test_pwm_status; }
inline void HAL_GPIO_Init(GPIO_TypeDef *, GPIO_InitTypeDef *) {}
inline void HAL_GPIO_WritePin(GPIO_TypeDef *, uint16_t, GPIO_PinState) {}
inline HAL_StatusTypeDef HAL_DMA_Abort(DMA_HandleTypeDef *) { return HAL_OK; }
inline HAL_StatusTypeDef HAL_SPI_Abort(SPI_HandleTypeDef *) { return HAL_OK; }
inline uint32_t __get_PRIMASK() { return 0; }
inline void __disable_irq() {}
inline void __enable_irq() {}
inline void __set_PRIMASK(uint32_t) {}
inline void __DMB() {}

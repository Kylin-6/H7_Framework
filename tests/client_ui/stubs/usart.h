#ifndef TEST_USART_H
#define TEST_USART_H
#include "stm32h7xx.h"
typedef struct
{
    void* hdmarx;
    void* hdmatx;
} UART_HandleTypeDef;
enum
{
    HAL_OK,
    HAL_ERROR,
    HAL_BUSY,
    HAL_TIMEOUT
};
#endif

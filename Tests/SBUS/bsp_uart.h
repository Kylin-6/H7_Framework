#ifndef TEST_BSP_UART_H
#define TEST_BSP_UART_H

#include "usart.h"

#include <cstdint>

using UART_Callback = void (*)(uint8_t *, uint16_t);

void UART_Init(UART_HandleTypeDef *huart, UART_Callback callback);

#endif

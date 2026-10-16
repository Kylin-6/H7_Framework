#ifndef TEST_BSP_UART_H
#define TEST_BSP_UART_H
#include "usart.h"
#ifdef __cplusplus
extern "C"
{
#endif
    void UART_Init(UART_HandleTypeDef* uart, void (*callback)(uint8_t*, uint16_t));
    uint8_t UART_Transmit_Data(UART_HandleTypeDef* uart, uint8_t* data, uint16_t length);
#ifdef __cplusplus
}
#endif
#endif

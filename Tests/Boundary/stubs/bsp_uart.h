#pragma once
#include <stdint.h>
#define UART_BUFFER_SIZE 512
enum { USART1 = 1, USART2, USART3, UART5, USART6, UART7, USART10 };
struct UART_HandleTypeDef { int Instance; };
struct Struct_UART_Manage_Object
{
    UART_HandleTypeDef *UART_Handler;
    uint8_t *Rx_Buffer_Ready;
};
extern Struct_UART_Manage_Object USART1_Manage_Object, USART2_Manage_Object,
    USART3_Manage_Object, UART5_Manage_Object, USART6_Manage_Object,
    UART7_Manage_Object, USART10_Manage_Object;
uint8_t UART_Transmit_Data(UART_HandleTypeDef *, uint8_t *, uint16_t);

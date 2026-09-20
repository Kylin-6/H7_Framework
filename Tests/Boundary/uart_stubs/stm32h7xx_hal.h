#pragma once
#include <stdint.h>

enum HAL_StatusTypeDef { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT };
enum { USART1 = 1, USART2, USART3, UART5, USART6, UART7, USART10 };
enum { HAL_UART_STATE_READY = 0x20, HAL_UART_STATE_BUSY_TX = 0x21 };
enum { HAL_DMA_STATE_READY = 1, HAL_DMA_STATE_BUSY = 2 };
enum { HAL_UART_ERROR_NONE, DMA_IT_HT, UART_CLEAR_IDLEF };
struct DMA_HandleTypeDef { uint32_t State; };
struct UART_HandleTypeDef
{
    int Instance;
    DMA_HandleTypeDef *hdmarx;
    DMA_HandleTypeDef *hdmatx;
    uint32_t gState;
    uint32_t ErrorCode;
};

uint32_t __get_PRIMASK();
void __disable_irq();
void __set_PRIMASK(uint32_t value);
void __DMB();
#define __HAL_DMA_DISABLE_IT(handle, flag) ((void)(handle), (void)(flag))
#define __HAL_UART_CLEAR_FLAG(handle, flag) ((void)(handle), (void)(flag))

HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *, const uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *, const uint8_t *, uint16_t, uint32_t);

#ifndef TEST_USART_H
#define TEST_USART_H

#include <cstdint>

#define UART_WORDLENGTH_9B 9U
#define UART_STOPBITS_2 2U
#define UART_PARITY_EVEN 2U
#define UART_MODE_RX 1U
#define UART_MODE_TX 2U

struct DMA_HandleTypeDef
{
    uint32_t unused;
};

struct UART_InitTypeDef
{
    uint32_t BaudRate;
    uint32_t WordLength;
    uint32_t StopBits;
    uint32_t Parity;
    uint32_t Mode;
};

struct UART_HandleTypeDef
{
    UART_InitTypeDef Init;
    DMA_HandleTypeDef *hdmarx;
};

extern "C" uint32_t HAL_GetTick(void);
extern "C" uint32_t __get_PRIMASK(void);
extern "C" void __disable_irq(void);
extern "C" void __set_PRIMASK(uint32_t value);
extern "C" void __DMB(void);

#endif

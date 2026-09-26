#pragma once
#include "stm32h7xx_hal.h"
#include "sys_timestamp.h"
#define SPI_BUFFER_SIZE 256
struct Struct_SPI_Manage_Object
{
    SPI_HandleTypeDef *SPI_Handler;
    uint8_t Tx_Buffer[SPI_BUFFER_SIZE], Rx_Buffer[SPI_BUFFER_SIZE];
    uint16_t Rx_Buffer_Length;
    GPIO_TypeDef *Activate_GPIOx;
    uint16_t Activate_GPIO_Pin, Tx_Buffer_Length;
    GPIO_PinState Activate_Level;
    unsigned Callback_Anomaly_Count;
    bool Transaction_Active;
};
extern Struct_SPI_Manage_Object SPI2_Manage_Object;
inline void SPI_Capture_Timeout_Snapshot(SPI_HandleTypeDef *, uint32_t) {}
uint8_t SPI_Transmit_Data(SPI_HandleTypeDef *, GPIO_TypeDef *, uint16_t,
                          GPIO_PinState, const uint8_t *, uint16_t);
uint8_t SPI_Transmit_Receive_Data(SPI_HandleTypeDef *, GPIO_TypeDef *, uint16_t,
                                  GPIO_PinState, const uint8_t *, uint16_t, uint16_t);

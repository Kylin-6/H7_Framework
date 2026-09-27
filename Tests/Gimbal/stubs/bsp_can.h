#pragma once
#include "fdcan.h"
#include "stm32h7xx.h"
struct Struct_CAN_Tx_Msg
{
    FDCAN_HandleTypeDef *hfdcan;
    uint32_t id;
    uint8_t data[8];
    uint8_t len;
};
using CAN_RxCallback_t = void (*)(FDCAN_HandleTypeDef *, uint32_t, uint8_t *, uint32_t, void *);
bool BSP_CAN_RegisterCallback(uint32_t, FDCAN_HandleTypeDef *, CAN_RxCallback_t, void *);
bool CAN_Tx_Submit(const Struct_CAN_Tx_Msg *);
bool CAN_Tx_Perform(const Struct_CAN_Tx_Msg *);

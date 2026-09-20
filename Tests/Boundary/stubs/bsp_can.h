#pragma once
#include <stdint.h>

struct FDCAN_HandleTypeDef { int instance; };
struct Struct_CAN_Tx_Msg
{
    FDCAN_HandleTypeDef *hfdcan;
    uint32_t id;
    uint8_t data[8];
    uint8_t len;
};
typedef void (*CAN_RxCallback_t)(FDCAN_HandleTypeDef *, uint32_t, uint8_t *, uint32_t, void *);
bool BSP_CAN_RegisterCallback(uint32_t, FDCAN_HandleTypeDef *, CAN_RxCallback_t, void *);
bool CAN_Tx_Submit(const Struct_CAN_Tx_Msg *);
bool CAN_Tx_Perform(const Struct_CAN_Tx_Msg *);

extern uint32_t test_irq_mask;
inline uint32_t __get_PRIMASK() { return test_irq_mask; }
inline void __disable_irq() { test_irq_mask = 1; }
inline void __enable_irq() { test_irq_mask = 0; }
inline void __DMB() {}

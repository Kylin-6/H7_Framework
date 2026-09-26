#pragma once
#include <stdint.h>

typedef struct { int instance; } FDCAN_HandleTypeDef;
typedef struct {
    uint32_t IdType, FilterIndex, FilterType, FilterConfig, FilterID1, FilterID2;
} FDCAN_FilterTypeDef;
typedef struct { uint32_t Identifier, DataLength, IdType, RxFrameType, FDFormat; } FDCAN_RxHeaderTypeDef;
typedef struct {
    uint32_t Identifier, IdType, TxFrameType, DataLength, ErrorStateIndicator;
    uint32_t BitRateSwitch, FDFormat, TxEventFifoControl, MessageMarker;
} FDCAN_TxHeaderTypeDef;
typedef int HAL_StatusTypeDef;

#define HAL_OK 0
#define FDCAN_STANDARD_ID 0
#define FDCAN_FILTER_MASK 0
#define FDCAN_FILTER_TO_RXFIFO0 0
#define FDCAN_REJECT 0
#define FDCAN_FILTER_REMOTE 0
#define FDCAN_REJECT_REMOTE 1
#define FDCAN_IT_RX_FIFO0_NEW_MESSAGE 1
#define FDCAN_RX_FIFO0 0
#define FDCAN_DATA_FRAME 0
#define FDCAN_ESI_ACTIVE 0
#define FDCAN_BRS_OFF 0
#define FDCAN_CLASSIC_CAN 0
#define FDCAN_NO_TX_EVENTS 0

HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *, FDCAN_FilterTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *, uint32_t, uint32_t, uint32_t, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *, uint32_t, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *);
uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *, uint32_t, FDCAN_RxHeaderTypeDef *, uint8_t *);
uint32_t HAL_FDCAN_GetTxFifoFreeLevel(FDCAN_HandleTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *, FDCAN_TxHeaderTypeDef *, const uint8_t *);
void Error_Handler(void);

extern uint32_t test_primask;
static inline uint32_t __get_PRIMASK(void) { return test_primask; }
static inline void __disable_irq(void) { test_primask = 1U; }
static inline void __enable_irq(void) { test_primask = 0U; }
static inline void __set_PRIMASK(uint32_t value) { test_primask = value; }
static inline void __DMB(void) {}

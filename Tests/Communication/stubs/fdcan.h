#pragma once
#include "stm32h7xx_hal.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef struct { int Instance; } FDCAN_HandleTypeDef;
typedef struct {
    uint32_t Identifier, IdType, RxFrameType, DataLength, FDFormat;
} FDCAN_RxHeaderTypeDef;
typedef struct {
    uint32_t Identifier, IdType, TxFrameType, DataLength, ErrorStateIndicator,
        BitRateSwitch, FDFormat, TxEventFifoControl, MessageMarker;
} FDCAN_TxHeaderTypeDef;
typedef struct {
    uint32_t IdType, FilterIndex, FilterType, FilterConfig, FilterID1, FilterID2;
} FDCAN_FilterTypeDef;
#define FDCAN_STANDARD_ID 0
#define FDCAN_EXTENDED_ID 0x40000000
#define FDCAN_DATA_FRAME 0
#define FDCAN_REMOTE_FRAME 0x20000000
#define FDCAN_CLASSIC_CAN 0
#define FDCAN_FD_CAN 0x200000
#define FDCAN_FILTER_MASK 2
#define FDCAN_FILTER_TO_RXFIFO0 1
#define FDCAN_REJECT 2
#define FDCAN_FILTER_REMOTE 0
#define FDCAN_REJECT_REMOTE 1
#define FDCAN_IT_RX_FIFO0_NEW_MESSAGE 1
#define FDCAN_RX_FIFO0 0
#define FDCAN_ESI_ACTIVE 0
#define FDCAN_BRS_OFF 0
#define FDCAN_NO_TX_EVENTS 0
#define FDCAN_DLC_BYTES_8 8
extern FDCAN_HandleTypeDef hfdcan1, hfdcan2, hfdcan3;
HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *, FDCAN_FilterTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *, uint32_t, uint32_t, uint32_t, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *, uint32_t, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *);
uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *, uint32_t, FDCAN_RxHeaderTypeDef *, uint8_t *);
uint32_t HAL_FDCAN_GetTxFifoFreeLevel(FDCAN_HandleTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *, const FDCAN_TxHeaderTypeDef *, const uint8_t *);
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *, uint32_t);
#ifdef __cplusplus
}
#endif

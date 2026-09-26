#pragma once
#include "stm32h7xx_hal.h"
#ifdef __cplusplus
extern "C" {
#endif
enum { USBD_OK, USBD_BUSY, USBD_FAIL };
enum {
    CDC_SEND_ENCAPSULATED_COMMAND, CDC_GET_ENCAPSULATED_RESPONSE,
    CDC_SET_COMM_FEATURE, CDC_GET_COMM_FEATURE, CDC_CLEAR_COMM_FEATURE,
    CDC_SET_LINE_CODING, CDC_GET_LINE_CODING, CDC_SET_CONTROL_LINE_STATE,
    CDC_SEND_BREAK
};
typedef struct { void *pClassData; } USBD_HandleTypeDef;
typedef struct {
    uint8_t *RxBuffer, *TxBuffer;
    uint32_t TxLength;
    volatile uint32_t TxState;
} USBD_CDC_HandleTypeDef;
typedef struct {
    int8_t (*Init)(void);
    int8_t (*DeInit)(void);
    int8_t (*Control)(uint8_t, uint8_t *, uint16_t);
    int8_t (*Receive)(uint8_t *, uint32_t *);
    int8_t (*TransmitCplt)(uint8_t *, uint32_t *, uint8_t);
} USBD_CDC_ItfTypeDef;
uint8_t USBD_CDC_SetRxBuffer(USBD_HandleTypeDef *, uint8_t *);
uint8_t USBD_CDC_SetTxBuffer(USBD_HandleTypeDef *, uint8_t *, uint32_t);
uint8_t USBD_CDC_ReceivePacket(USBD_HandleTypeDef *);
uint8_t USBD_CDC_TransmitPacket(USBD_HandleTypeDef *);
#ifdef __cplusplus
}
#endif

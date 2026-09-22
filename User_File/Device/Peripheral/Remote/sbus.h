/**
 * @file sbus.h
 * @brief Generic S.BUS receiver driver, validated with FlySky FS-i6X/FS-iA6B.
 */
#ifndef SBUS_H
#define SBUS_H

#include "usart.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SBUS_FRAME_SIZE 25U
#define SBUS_CHANNEL_COUNT 16U
#define SBUS_CHANNEL_OFFSET 1024
#define SBUS_RX_TIMEOUT_MS 100U

typedef struct
{
    int16_t channels[SBUS_CHANNEL_COUNT];
    uint8_t channel_17;
    uint8_t channel_18;
    uint8_t frame_lost;
    uint8_t failsafe;
    uint32_t sequence;
    uint32_t timestamp_ms;
} Struct_SBUS_Frame;

typedef struct
{
    uint32_t rx_events;
    uint32_t rx_bytes;
    uint32_t valid_frames;
    uint32_t invalid_candidates;
    uint32_t resync_events;
    uint32_t frame_lost_frames;
    uint32_t failsafe_frames;
    uint16_t last_rx_size;
} Struct_SBUS_Diagnostics;

bool SBUS_Init(UART_HandleTypeDef *huart);
bool SBUS_ReadLatest(Struct_SBUS_Frame *frame);
bool SBUS_IsOnline(void);
bool SBUS_IsHealthy(void);
void SBUS_GetDiagnostics(Struct_SBUS_Diagnostics *diagnostics);
void SBUS_RxCallback(uint8_t *buffer, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif

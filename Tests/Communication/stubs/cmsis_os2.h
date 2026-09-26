#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
typedef void *osMessageQueueId_t;
typedef enum { osOK, osErrorResource } osStatus_t;
osMessageQueueId_t osMessageQueueNew(uint32_t, uint32_t, const void *);
osStatus_t osMessageQueuePut(osMessageQueueId_t, const void *, uint8_t, uint32_t);
osStatus_t osMessageQueueGet(osMessageQueueId_t, void *, uint8_t *, uint32_t);
#ifdef __cplusplus
}
#endif

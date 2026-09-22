#pragma once
#include <stdint.h>

typedef void *osMessageQueueId_t;
typedef int32_t osStatus_t;
#define osOK 0

osMessageQueueId_t osMessageQueueNew(uint32_t count, uint32_t size, const void *attr);
osStatus_t osMessageQueuePut(osMessageQueueId_t queue, const void *message,
                             uint8_t priority, uint32_t timeout);
osStatus_t osMessageQueueGet(osMessageQueueId_t queue, void *message,
                             uint8_t *priority, uint32_t timeout);

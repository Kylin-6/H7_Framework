#ifndef DYNAMIC_MESSAGE_CENTER_H
#define DYNAMIC_MESSAGE_CENTER_H

#include <stdbool.h>
#include <stdint.h>

#define DYNAMIC_MESSAGE_CENTER_MAX_TOPIC_NAME_LENGTH 31U
#define DYNAMIC_MESSAGE_CENTER_MAX_MESSAGE_SIZE 256U

#ifdef __cplusplus
extern "C" {
#endif

typedef struct DynamicPublisher DynamicPublisher_t;
typedef struct DynamicSubscriber DynamicSubscriber_t;

/**
 * @brief Enable the dynamic message center during the pre-scheduler phase.
 * @return true when called after osKernelInitialize() and before osKernelStart().
 */
bool DynamicMessageCenter_Init(void);

/**
 * @brief Register the single publisher that owns a named topic.
 * @return Publisher handle, or NULL for invalid input, a size conflict, a
 *         duplicate publisher, closed registration, or allocation failure.
 */
DynamicPublisher_t *DynamicPublisher_Register(const char *topic_name,
                                              uint16_t message_size);

/**
 * @brief Register an independent latest-value subscriber queue.
 * @return Subscriber handle, or NULL on validation or allocation failure.
 * @note A subscriber may be registered before its topic's publisher.
 */
DynamicSubscriber_t *DynamicSubscriber_Register(const char *topic_name,
                                                uint16_t message_size);

/**
 * @brief Overwrite every subscriber's one-element queue with the latest data.
 * @return Number of subscriber queues successfully updated. Zero is valid when
 *         the topic has no subscribers, and is also returned for invalid input.
 */
uint32_t DynamicPublisher_Publish(DynamicPublisher_t *publisher,
                                  const void *data);

/**
 * @brief Non-blocking read that consumes this subscriber's latest unread data.
 * @return true when data was read; false leaves the destination unchanged.
 */
bool DynamicSubscriber_Read(DynamicSubscriber_t *subscriber, void *data);

#ifdef __cplusplus
}
#endif

#endif

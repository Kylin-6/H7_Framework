/**
 * @file dynamic_message_center.c
 * @brief Low-rate dynamic publish/subscribe channel for application messages.
 *
 * @details The registration model is adapted from the MIT-licensed
 * basic_framework and Meta-Embedded-NG message centers. This implementation
 * uses the project FreeRTOS heap_5 allocator, rejects runtime registration,
 * and keeps one latest-value queue per subscriber.
 */

#include "dynamic_message_center.h"

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "queue.h"
#include "stm32h7xx.h"
#include "task.h"
#include <stddef.h>

struct DynamicSubscriber
{
    QueueHandle_t queue;
    struct DynamicSubscriber *next;
};

struct DynamicPublisher
{
    char topic_name[DYNAMIC_MESSAGE_CENTER_MAX_TOPIC_NAME_LENGTH + 1U];
    uint16_t message_size;
    bool publisher_registered;
    struct DynamicSubscriber *first_subscriber;
    struct DynamicPublisher *next;
};

static DynamicPublisher_t *Dynamic_Topic_List = NULL;
static bool Dynamic_Message_Center_Initialized = false;

static bool DynamicMessageCenter_RegistrationIsOpen(void)
{
    return Dynamic_Message_Center_Initialized &&
           (__get_IPSR() == 0U) &&
           (osKernelGetState() == osKernelReady) &&
           (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED);
}

static bool DynamicMessageCenter_ValidateTopicName(const char *topic_name,
                                                   size_t *name_length)
{
    if (topic_name == NULL || name_length == NULL)
    {
        return false;
    }

    size_t length = 0U;
    while (length <= DYNAMIC_MESSAGE_CENTER_MAX_TOPIC_NAME_LENGTH &&
           topic_name[length] != '\0')
    {
        length++;
    }

    if (length == 0U || length > DYNAMIC_MESSAGE_CENTER_MAX_TOPIC_NAME_LENGTH)
    {
        return false;
    }

    *name_length = length;
    return true;
}

static bool DynamicMessageCenter_ValidateMessageSize(uint16_t message_size)
{
    return message_size > 0U &&
           message_size <= DYNAMIC_MESSAGE_CENTER_MAX_MESSAGE_SIZE;
}

static bool DynamicMessageCenter_TopicNameMatches(
    const DynamicPublisher_t *topic, const char *topic_name, size_t name_length)
{
    size_t index = 0U;
    while (index < name_length && topic->topic_name[index] == topic_name[index])
    {
        index++;
    }
    return index == name_length && topic->topic_name[index] == '\0';
}

static DynamicPublisher_t *DynamicMessageCenter_FindTopic(
    const char *topic_name, size_t name_length)
{
    DynamicPublisher_t *topic = Dynamic_Topic_List;
    while (topic != NULL)
    {
        if (DynamicMessageCenter_TopicNameMatches(topic, topic_name, name_length))
        {
            return topic;
        }
        topic = topic->next;
    }
    return NULL;
}

static DynamicPublisher_t *DynamicMessageCenter_CreateTopic(
    const char *topic_name, size_t name_length, uint16_t message_size)
{
    DynamicPublisher_t *topic =
        (DynamicPublisher_t *)pvPortMalloc(sizeof(DynamicPublisher_t));
    if (topic == NULL)
    {
        return NULL;
    }

    for (size_t index = 0U; index < name_length; index++)
    {
        topic->topic_name[index] = topic_name[index];
    }
    topic->topic_name[name_length] = '\0';
    topic->message_size = message_size;
    topic->publisher_registered = false;
    topic->first_subscriber = NULL;
    topic->next = NULL;
    return topic;
}

static void DynamicMessageCenter_InsertTopic(DynamicPublisher_t *topic)
{
    topic->next = Dynamic_Topic_List;
    Dynamic_Topic_List = topic;
}

bool DynamicMessageCenter_Init(void)
{
    if (__get_IPSR() != 0U ||
        osKernelGetState() != osKernelReady ||
        xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
        return false;
    }

    if (Dynamic_Message_Center_Initialized)
    {
        return true;
    }

    Dynamic_Topic_List = NULL;
    Dynamic_Message_Center_Initialized = true;
    return true;
}

DynamicPublisher_t *DynamicPublisher_Register(const char *topic_name,
                                              uint16_t message_size)
{
    size_t name_length = 0U;
    if (!DynamicMessageCenter_RegistrationIsOpen() ||
        !DynamicMessageCenter_ValidateTopicName(topic_name, &name_length) ||
        !DynamicMessageCenter_ValidateMessageSize(message_size))
    {
        return NULL;
    }

    DynamicPublisher_t *topic =
        DynamicMessageCenter_FindTopic(topic_name, name_length);
    if (topic != NULL)
    {
        if (topic->message_size != message_size || topic->publisher_registered)
        {
            return NULL;
        }
        topic->publisher_registered = true;
        return topic;
    }

    topic = DynamicMessageCenter_CreateTopic(topic_name, name_length, message_size);
    if (topic == NULL)
    {
        return NULL;
    }

    topic->publisher_registered = true;
    DynamicMessageCenter_InsertTopic(topic);
    return topic;
}

DynamicSubscriber_t *DynamicSubscriber_Register(const char *topic_name,
                                                uint16_t message_size)
{
    size_t name_length = 0U;
    if (!DynamicMessageCenter_RegistrationIsOpen() ||
        !DynamicMessageCenter_ValidateTopicName(topic_name, &name_length) ||
        !DynamicMessageCenter_ValidateMessageSize(message_size))
    {
        return NULL;
    }

    DynamicPublisher_t *topic =
        DynamicMessageCenter_FindTopic(topic_name, name_length);
    const bool new_topic = topic == NULL;
    if (!new_topic && topic->message_size != message_size)
    {
        return NULL;
    }

    if (new_topic)
    {
        topic = DynamicMessageCenter_CreateTopic(topic_name, name_length,
                                                 message_size);
        if (topic == NULL)
        {
            return NULL;
        }
    }

    DynamicSubscriber_t *subscriber =
        (DynamicSubscriber_t *)pvPortMalloc(sizeof(DynamicSubscriber_t));
    if (subscriber == NULL)
    {
        if (new_topic)
        {
            vPortFree(topic);
        }
        return NULL;
    }

    subscriber->queue = xQueueCreate(1U, message_size);
    if (subscriber->queue == NULL)
    {
        vPortFree(subscriber);
        if (new_topic)
        {
            vPortFree(topic);
        }
        return NULL;
    }

    subscriber->next = topic->first_subscriber;
    topic->first_subscriber = subscriber;
    if (new_topic)
    {
        DynamicMessageCenter_InsertTopic(topic);
    }
    return subscriber;
}

uint32_t DynamicPublisher_Publish(DynamicPublisher_t *publisher,
                                  const void *data)
{
    if (!Dynamic_Message_Center_Initialized || __get_IPSR() != 0U ||
        publisher == NULL || data == NULL || !publisher->publisher_registered)
    {
        return 0U;
    }

    uint32_t publish_count = 0U;
    DynamicSubscriber_t *subscriber = publisher->first_subscriber;
    while (subscriber != NULL)
    {
        if (xQueueOverwrite(subscriber->queue, data) == pdPASS)
        {
            publish_count++;
        }
        subscriber = subscriber->next;
    }
    return publish_count;
}

bool DynamicSubscriber_Read(DynamicSubscriber_t *subscriber, void *data)
{
    if (!Dynamic_Message_Center_Initialized || __get_IPSR() != 0U ||
        subscriber == NULL || data == NULL)
    {
        return false;
    }

    return xQueueReceive(subscriber->queue, data, 0U) == pdPASS;
}

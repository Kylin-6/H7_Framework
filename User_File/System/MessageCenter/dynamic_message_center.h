#ifndef DYNAMIC_MESSAGE_CENTER_H
#define DYNAMIC_MESSAGE_CENTER_H

#include <stdbool.h>
#include <stdint.h>

/** Topic 名称最大有效字符数，不包含字符串结尾的 '\0'。 */
#define DYNAMIC_MESSAGE_CENTER_MAX_TOPIC_NAME_LENGTH 31U
/** 动态通道允许的单条消息最大字节数。 */
#define DYNAMIC_MESSAGE_CENTER_MAX_MESSAGE_SIZE 256U

#ifdef __cplusplus
extern "C" {
#endif

typedef struct DynamicPublisher DynamicPublisher_t;
typedef struct DynamicSubscriber DynamicSubscriber_t;

/**
 * @brief 在调度器启动前初始化动态消息中心。
 * @return 仅当调用发生在 osKernelInitialize() 之后、osKernelStart() 之前时返回 true。
 * @note 不创建任务；未注册 Topic 时不消耗动态内存。
 */
bool DynamicMessageCenter_Init(void);

/**
 * @brief 注册指定 Topic 的唯一发布者。
 * @return 成功时返回 Publisher 句柄；参数非法、尺寸冲突、重复发布者、
 *         注册窗口关闭或内存不足时返回 NULL。
 * @note 只允许在调度器启动前调用。
 */
DynamicPublisher_t *DynamicPublisher_Register(const char *topic_name,
                                              uint16_t message_size);

/**
 * @brief 为订阅者注册一个独立的 Latest-Value 队列。
 * @return 成功时返回 Subscriber 句柄；校验或分配失败时返回 NULL。
 * @note Subscriber 可以先于同名 Topic 的 Publisher 注册。
 */
DynamicSubscriber_t *DynamicSubscriber_Register(const char *topic_name,
                                                uint16_t message_size);

/**
 * @brief 用最新数据覆盖该 Topic 下所有订阅者的一元素队列。
 * @return 成功写入的订阅者队列数量。没有订阅者时返回 0 属于正常情况；
 *         参数或句柄非法时同样返回 0。
 * @note 仅供任务上下文调用，不提供 ISR 发布接口。
 */
uint32_t DynamicPublisher_Publish(DynamicPublisher_t *publisher,
                                  const void *data);

/**
 * @brief 非阻塞读取并消费该订阅者尚未读取的最新数据。
 * @return 读到数据时返回 true；返回 false 时不会修改输出对象。
 */
bool DynamicSubscriber_Read(DynamicSubscriber_t *subscriber, void *data);

#ifdef __cplusplus
}
#endif

#endif

#ifndef MESSAGE_CENTER_TOPIC_H
#define MESSAGE_CENTER_TOPIC_H

#include "stm32h7xx.h"
#include <cstdint>
#include <type_traits>

extern "C" uint64_t SYS_Timestamp_Get_Microsecond(void);

/** 静态 Topic 一次发布的完整快照，数据与元信息属于同一帧。 */
template<typename T>
struct TopicSnapshot
{
    T data{};                    ///< 消息数据
    uint32_t sequence = 0U;      ///< 发布序号，每次 Publish 自增
    uint64_t timestamp_us = 0U;  ///< 发布时刻，单位：us
    bool valid = false;          ///< 是否至少发布过一次
};

/**
 * @brief 静态、非阻塞的 Latest-Value Topic。
 * @tparam T 体积较小且可平凡复制的消息类型。
 *
 * Publish/Read 面向任务上下文。通过极短的 PRIMASK 临界区保证 FreeRTOS
 * 任务间读取的数据与元信息一致，不使用互斥锁、队列或动态内存。
 * 适用于只关心最新值的连续状态和控制目标，与消息更新频率无关。
 */
template<typename T>
class Topic
{
    static_assert(std::is_trivially_copyable<T>::value,
                  "Topic messages must be trivially copyable");

public:
    /** 发布一个新状态；旧状态直接被覆盖，不保存历史记录。 */
    void Publish(const T &data)
    {
        const uint64_t timestamp = SYS_Timestamp_Get_Microsecond();
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();

        /* 数据和元信息必须在同一临界区内更新，避免读到半帧数据。 */
        data_ = data;
        sequence_++;
        timestamp_ = timestamp;
        valid_ = true;
        __DMB();

        __set_PRIMASK(primask);
    }

    bool Read(T &data) const
    {
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();

        /* Topic 尚未首次发布时保持调用者的输出对象不变。 */
        const bool valid = valid_;
        if (valid)
        {
            data = data_;
        }
        __DMB();

        __set_PRIMASK(primask);
        return valid;
    }

    /**
     * @brief 一次性读取同一发布帧的数据、序号和时间戳。
     * @return 在一个短临界区中取得的一致快照。
     * @note 需要关联序号与时间戳时优先使用本接口，避免分开读取产生 TOCTOU。
     */
    TopicSnapshot<T> ReadWithMeta() const
    {
        TopicSnapshot<T> snapshot;
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();

        snapshot.data = data_;
        snapshot.sequence = sequence_;
        snapshot.timestamp_us = timestamp_;
        snapshot.valid = valid_;
        __DMB();

        __set_PRIMASK(primask);
        return snapshot;
    }

    uint32_t Sequence() const
    {
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        const uint32_t sequence = sequence_;
        __DMB();
        __set_PRIMASK(primask);
        return sequence;
    }

    uint64_t Timestamp() const
    {
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        const uint64_t timestamp = timestamp_;
        __DMB();
        __set_PRIMASK(primask);
        return timestamp;
    }

private:
    T data_{};                   ///< 最近一次发布的数据
    uint32_t sequence_ = 0U;     ///< 发布序号
    uint64_t timestamp_ = 0U;    ///< 最近一次发布时间，单位：us
    bool valid_ = false;         ///< 首次发布完成标志
};

/** Topic 的轻量发布端；仅保存绑定对象的引用。 */
template<typename T>
class Publisher
{
public:
    explicit Publisher(Topic<T> &topic) : topic_(topic) {}

    void Publish(const T &data)
    {
        topic_.Publish(data);
    }

private:
    Topic<T> &topic_;
};

/** 每个订阅端独立跟踪序号，只在有新发布时返回数据。 */
template<typename T>
class Subscriber
{
public:
    explicit Subscriber(Topic<T> &topic) : topic_(topic) {}

    bool Read(T &data)
    {
        const TopicSnapshot<T> snapshot = topic_.ReadWithMeta();
        if (!snapshot.valid ||
            (has_read_ && snapshot.sequence == last_sequence_))
        {
            return false;
        }

        data = snapshot.data;
        last_sequence_ = snapshot.sequence;
        has_read_ = true;
        return true;
    }

private:
    Topic<T> &topic_;
    uint32_t last_sequence_ = 0U;
    bool has_read_ = false;
};

#endif

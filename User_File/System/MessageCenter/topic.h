#ifndef MESSAGE_CENTER_TOPIC_H
#define MESSAGE_CENTER_TOPIC_H

#include "stm32h7xx.h"
#include <cstdint>
#include <type_traits>

extern "C" uint64_t SYS_Timestamp_Get_Microsecond(void);

template<typename T>
struct TopicSnapshot
{
    T data{};
    uint32_t sequence = 0U;
    uint64_t timestamp_us = 0U;
    bool valid = false;
};

/**
 * @brief Static, non-blocking latest-value topic.
 * @tparam T Small trivially-copyable message type.
 *
 * Publish and Read are intended for task context. A short PRIMASK critical
 * section makes the data and metadata snapshot consistent across FreeRTOS
 * tasks without a mutex, queue, or dynamic allocation.
 */
template<typename T>
class Topic
{
    static_assert(std::is_trivially_copyable<T>::value,
                  "Topic messages must be trivially copyable");

public:
    void Publish(const T &data)
    {
        const uint64_t timestamp = SYS_Timestamp_Get_Microsecond();
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();

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
     * @brief Read data and metadata from the same published frame.
     * @return A consistent snapshot captured in one short critical section.
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
    T data_{};
    uint32_t sequence_ = 0U;
    uint64_t timestamp_ = 0U;
    bool valid_ = false;
};

#endif

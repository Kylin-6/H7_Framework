#ifndef MESSAGE_CENTER_EVENT_QUEUE_H
#define MESSAGE_CENTER_EVENT_QUEUE_H

#include "stm32h7xx.h"
#include <cstddef>
#include <cstdint>
#include <type_traits>

/** 静态、非阻塞、线程安全的固定容量 FIFO。 */
template<typename T, size_t N>
class EventQueue
{
    static_assert(N > 0U, "EventQueue capacity must be greater than zero");
    static_assert(std::is_trivially_copyable<T>::value,
                  "EventQueue messages must be trivially copyable");

public:
    bool Push(const T &event)
    {
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();

        if (size_ == N)
        {
            overflow_count_++;
            __DMB();
            __set_PRIMASK(primask);
            return false;
        }

        storage_[tail_] = event;
        tail_ = (tail_ + 1U) % N;
        size_++;
        __DMB();
        __set_PRIMASK(primask);
        return true;
    }

    bool Pop(T &event)
    {
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();

        if (size_ == 0U)
        {
            __DMB();
            __set_PRIMASK(primask);
            return false;
        }

        event = storage_[head_];
        head_ = (head_ + 1U) % N;
        size_--;
        __DMB();
        __set_PRIMASK(primask);
        return true;
    }

    bool Empty() const
    {
        return Size() == 0U;
    }

    size_t Size() const
    {
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        const size_t size = size_;
        __DMB();
        __set_PRIMASK(primask);
        return size;
    }

    uint32_t OverflowCount() const
    {
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        const uint32_t count = overflow_count_;
        __DMB();
        __set_PRIMASK(primask);
        return count;
    }

private:
    T storage_[N]{};
    size_t head_ = 0U;
    size_t tail_ = 0U;
    size_t size_ = 0U;
    uint32_t overflow_count_ = 0U;
};

#endif

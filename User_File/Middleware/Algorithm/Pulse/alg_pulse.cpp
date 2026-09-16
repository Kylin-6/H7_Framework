#include "alg_pulse.h"

/**
 * @brief Dispatch a compile-time callback table from a 1 ms task tick.
 * @note No registration, allocation, or variadic argument decoding occurs.
 */
void Pulse_Dispatch(const PulseEntry_t *entries, size_t entry_count,
                    uint32_t tick_ms)
{
    if (entries == nullptr)
    {
        return;
    }

    for (size_t index = 0U; index < entry_count; ++index)
    {
        const PulseEntry_t &entry = entries[index];
        if (entry.period_ms != 0U && entry.callback != nullptr &&
            tick_ms % entry.period_ms == 0U)
        {
            entry.callback();
        }
    }
}

/**
 * @file    alg_pulse.h
 * @brief   Pulse algorithm module template
 */

#ifndef ALG_PULSE_H
#define ALG_PULSE_H
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef void (*pulse_cb_t)(void);

    typedef struct
    {
        uint16_t period_ms;
        pulse_cb_t callback;
    } PulseEntry_t;

    void Pulse_Dispatch(const PulseEntry_t *entries, size_t entry_count,
                        uint32_t tick_ms);

#ifdef __cplusplus
}
#endif

#endif /* ALG_PULSE_H */

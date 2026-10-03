/**
 * @file    alg_pulse.h
 * @brief   在调用者上下文按毫秒调度轮次分发静态周期回调表
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

    /** tick_ms 为调用方维护的调度轮次；0 时触发所有有效项。
     * 不计时、不补执行漏掉的轮次；回调同步执行，必须控制总耗时。 */
    void Pulse_Dispatch(const PulseEntry_t *entries, size_t entry_count,
                        uint32_t tick_ms);

#ifdef __cplusplus
}
#endif

#endif /* ALG_PULSE_H */

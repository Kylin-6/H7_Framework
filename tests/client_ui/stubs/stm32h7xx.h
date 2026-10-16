#ifndef TEST_STM32H7XX_H
#define TEST_STM32H7XX_H
#include <stdint.h>
#ifdef __cplusplus
extern "C"
{
#endif
    extern uint32_t test_primask;
    static inline uint32_t __get_PRIMASK(void)
    {
        return test_primask;
    }
    static inline void __disable_irq(void)
    {
        test_primask = 1U;
    }
    static inline void __set_PRIMASK(uint32_t value)
    {
        test_primask = value;
    }
    static inline void __DMB(void)
    {
    }
    uint32_t HAL_GetTick(void);
#ifdef __cplusplus
}
#endif
#endif

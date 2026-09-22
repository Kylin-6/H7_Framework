#pragma once

#include <stdint.h>

extern uint32_t test_irq_mask;

inline uint32_t __get_PRIMASK() { return test_irq_mask; }
inline void __disable_irq() { test_irq_mask = 1U; }
inline void __set_PRIMASK(uint32_t value) { test_irq_mask = value; }
inline void __DMB() {}

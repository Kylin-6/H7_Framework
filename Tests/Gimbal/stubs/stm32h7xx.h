#pragma once
#include <stdint.h>
extern uint32_t test_irq_mask;
inline uint32_t __get_PRIMASK() { return test_irq_mask; }
inline void __disable_irq() { test_irq_mask = 1; }
inline void __enable_irq() { test_irq_mask = 0; }
inline void __set_PRIMASK(uint32_t value) { test_irq_mask = value; }
inline void __DMB() {}

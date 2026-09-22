#pragma once
#include <stdint.h>

extern uint32_t test_primask;
inline uint32_t __get_PRIMASK() { return test_primask; }
inline void __disable_irq() { test_primask = 1U; }
inline void __set_PRIMASK(uint32_t value) { test_primask = value; }
inline void __DMB() {}

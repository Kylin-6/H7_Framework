#pragma once
#include "sys_timestamp.h"
typedef void *osThreadId_t;
inline uint32_t osThreadFlagsSet(osThreadId_t, uint32_t flags) { return flags; }
inline void osDelay(uint32_t milliseconds) { Namespace_SYS_Timestamp::Delay_Millisecond(milliseconds); }

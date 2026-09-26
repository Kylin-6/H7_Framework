#pragma once
#include <stdint.h>
extern uint64_t test_time_us;
struct Class_Timestamp
{
    uint64_t Get_Now_Microsecond() const { return test_time_us; }
    uint64_t Get_Current_Timestamp() const { return test_time_us; }
};
extern Class_Timestamp SYS_Timestamp;
namespace Namespace_SYS_Timestamp { void Delay_Millisecond(uint32_t milliseconds); }

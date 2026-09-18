#pragma once
#include <stdint.h>
extern uint64_t test_timestamp_us;
class Class_Timestamp
{
public:
    uint64_t Get_Now_Microsecond() const { return test_timestamp_us; }
};
extern Class_Timestamp SYS_Timestamp;

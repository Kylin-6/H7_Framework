#pragma once
#include <stdint.h>
class Class_Timestamp
{
public:
    uint64_t Get_Current_Timestamp() const { return 0; }
};
extern Class_Timestamp SYS_Timestamp;

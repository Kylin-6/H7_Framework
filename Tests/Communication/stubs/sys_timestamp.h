#pragma once
#include <stdint.h>
struct Class_Timestamp {
    uint64_t Get_Current_Timestamp() const { return 123456; }
};
extern Class_Timestamp SYS_Timestamp;

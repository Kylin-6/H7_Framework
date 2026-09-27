#include "board_config.h"

const BoardHardware &BoardConfig_Get(void)
{
    static const BoardHardware hardware{
        nullptr, nullptr, &hfdcan1, &hfdcan2, nullptr,
        false, false, true, true, false, false};
    return hardware;
}

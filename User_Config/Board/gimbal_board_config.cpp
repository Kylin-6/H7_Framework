#include "board_config.h"

const BoardHardware &BoardConfig_Get(void)
{
    static const BoardHardware hardware{
        &hfdcan1, &hfdcan1, nullptr, nullptr, &hfdcan3,
        true, true, true, true, true, true};
    return hardware;
}

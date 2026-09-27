#include "board_config.h"

const BoardHardware &BoardConfig_Get(void)
{
    static const BoardHardware hardware{
        &hfdcan2, &hfdcan1, &hfdcan1, &hfdcan2, &hfdcan3,
        true, true, true, true, true, true};
    return hardware;
}

#ifndef H7_BOARD_CONFIG_H
#define H7_BOARD_CONFIG_H

#include "fdcan.h"

/** Hardware fitted to this firmware target; transport mapping lives separately. */
struct BoardHardware
{
    FDCAN_HandleTypeDef *gimbal_yaw_bus;
    FDCAN_HandleTypeDef *gimbal_pitch_bus;
    FDCAN_HandleTypeDef *chassis_wheel_bus;
    FDCAN_HandleTypeDef *chassis_steer_bus;
    FDCAN_HandleTypeDef *shoot_bus;
    bool imu;
    bool flash;
    bool adc;
    bool power;
    bool indicators;
    bool usb_debug;
};

const BoardHardware &BoardConfig_Get(void);

#endif

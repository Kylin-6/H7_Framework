#ifndef H7_BOARD_CONFIG_H
#define H7_BOARD_CONFIG_H

#include "fdcan.h"

/** 当前构建目标的物理硬件资源；不决定应用源码，也不保存板间通信路由。 */
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

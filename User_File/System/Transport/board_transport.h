#ifndef BOARD_TRANSPORT_H
#define BOARD_TRANSPORT_H

#include "message_types.h"

/** 仅由当前 CMake 板型选中的固定协议绑定实现；不是动态 Router。 */
void BoardTransport_Init(void);
void BoardTransport_Poll(void);
void BoardTransport_SendChassis(const ChassisCmd &command);

#endif

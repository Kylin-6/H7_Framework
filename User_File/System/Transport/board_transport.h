#ifndef BOARD_TRANSPORT_H
#define BOARD_TRANSPORT_H

#include "message_types.h"

/** 仅由当前 CMake 板型选中的固定协议绑定实现；不是动态 Router。 */
bool BoardTransport_Init(void); ///< 回调或静态 Daemon 注册失败时返回 false。
bool BoardTransport_IsOnline(void); ///< 合法且及时的接收流活性，不代表业务 Topic 新鲜。
uint32_t BoardTransport_OfflineDurationMs(void);
void BoardTransport_Poll(void);
void BoardTransport_SendChassis(const ChassisCmd &command);

#endif

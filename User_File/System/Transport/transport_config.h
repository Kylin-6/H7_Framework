#ifndef BOARD_TRANSPORT_CONFIG_H
#define BOARD_TRANSPORT_CONFIG_H

#include "fdcan.h"
#include "transport_protocol.h"

namespace TransportConfig
{
constexpr BoardId kCommandSource = BoardId::Gimbal;
constexpr BoardId kCommandTarget = BoardId::Chassis;
constexpr uint16_t kChassisCmdCanId = TransportProtocol::CanId(
    kCommandSource, kCommandTarget, MessageId::ChassisCmd);
}

/** One fixed CAN link per firmware target; no runtime routing table. */
FDCAN_HandleTypeDef *TransportConfig_Bus(void);

#endif

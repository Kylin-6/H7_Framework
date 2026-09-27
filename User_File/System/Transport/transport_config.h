#ifndef BOARD_TRANSPORT_CONFIG_H
#define BOARD_TRANSPORT_CONFIG_H

#include "fdcan.h"
#include "transport_protocol.h"

namespace TransportConfig
{
// 固定板间消息方向与标准 CAN ID；CMake 决定编入哪块板，BoardConfig 管硬件接线。
constexpr BoardId kCommandSource = BoardId::Gimbal;
constexpr BoardId kCommandTarget = BoardId::Chassis;
constexpr uint16_t kChassisCmdCanId = TransportProtocol::CanId(
    kCommandSource, kCommandTarget, MessageId::ChassisCmd);
constexpr uint16_t kChassisFeedbackCanId = TransportProtocol::CanId(
    BoardId::Chassis, BoardId::Gimbal, MessageId::ChassisFeedback);
static_assert(kChassisFeedbackCanId == 0x222U, "fixed feedback CAN ID");
}

/** One fixed CAN link per firmware target; no runtime routing table. */
FDCAN_HandleTypeDef *TransportConfig_Bus(void);

#endif

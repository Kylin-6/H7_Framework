#ifndef BOARD_TRANSPORT_PROTOCOL_H
#define BOARD_TRANSPORT_PROTOCOL_H

#include "message_types.h"

#include <stdint.h>

enum class BoardId : uint8_t { Gimbal = 1U, Chassis = 2U };
enum class MessageId : uint8_t { ChassisCmd = 1U, ChassisFeedback = 2U };

namespace TransportProtocol
{
constexpr uint8_t kVersion = 1U;
constexpr uint8_t kPayloadSize = 8U;
constexpr uint32_t kCommandMaxAgeUs = 100000U;
constexpr uint16_t CanId(BoardId source, BoardId target, MessageId message)
{
    return (static_cast<uint16_t>(source) << 8U) |
           (static_cast<uint16_t>(target) << 5U) |
           static_cast<uint16_t>(message);
}
bool EncodeChassisCmd(const ChassisCmd &command, uint8_t sequence,
                      uint8_t (&bytes)[kPayloadSize]);
bool DecodeChassisCmd(const uint8_t *bytes, uint32_t size,
                      ChassisCmd &command, uint8_t &sequence);
bool EncodeChassisFeedback(const ChassisFeedback &feedback, uint8_t sequence,
                           uint8_t (&bytes)[kPayloadSize]);
bool DecodeChassisFeedback(const uint8_t *bytes, uint32_t size,
                           ChassisFeedback &feedback, uint8_t &sequence);
bool SequenceNewer(uint8_t candidate, uint8_t previous);
}

#endif

#include "transport_protocol.h"

#include <cmath>

namespace
{
bool EncodeScalar(float value, uint8_t *bytes)
{
    const float scaled = value * 1000.0f;
    if (!std::isfinite(scaled) || scaled < -32768.0f || scaled > 32767.0f)
    {
        return false;
    }
    const int16_t fixed = static_cast<int16_t>(std::lround(scaled));
    const uint16_t bits = static_cast<uint16_t>(fixed);
    bytes[0] = static_cast<uint8_t>(bits);
    bytes[1] = static_cast<uint8_t>(bits >> 8U);
    return true;
}

float DecodeScalar(const uint8_t *bytes)
{
    const uint16_t bits = static_cast<uint16_t>(bytes[0]) |
                          (static_cast<uint16_t>(bytes[1]) << 8U);
    const int32_t signed_value = bits < 0x8000U ? bits :
                                 static_cast<int32_t>(bits) - 0x10000;
    return static_cast<float>(signed_value) * 0.001f;
}
}

namespace TransportProtocol
{
bool EncodeChassisCmd(const ChassisCmd &command, uint8_t sequence,
                      uint8_t (&bytes)[kPayloadSize])
{
    const uint8_t mode = static_cast<uint8_t>(command.mode);
    if (mode > static_cast<uint8_t>(ChassisMode::ROTATE))
    {
        return false;
    }
    uint8_t encoded[kPayloadSize] = {sequence,
        static_cast<uint8_t>((kVersion << 4U) | mode)};
    if (!EncodeScalar(command.velocity_x_m_s, &encoded[2]) ||
        !EncodeScalar(command.velocity_y_m_s, &encoded[4]) ||
        !EncodeScalar(command.angular_velocity_rad_s, &encoded[6]))
    {
        return false;
    }
    for (uint8_t index = 0U; index < kPayloadSize; ++index)
    {
        bytes[index] = encoded[index];
    }
    return true;
}

bool DecodeChassisCmd(const uint8_t *bytes, uint32_t size,
                      ChassisCmd &command, uint8_t &sequence)
{
    if (bytes == nullptr || size != kPayloadSize ||
        (bytes[1] >> 4U) != kVersion ||
        (bytes[1] & 0x0FU) > static_cast<uint8_t>(ChassisMode::ROTATE))
    {
        return false;
    }
    ChassisCmd decoded{};
    decoded.mode = static_cast<ChassisMode>(bytes[1] & 0x0FU);
    decoded.velocity_x_m_s = DecodeScalar(&bytes[2]);
    decoded.velocity_y_m_s = DecodeScalar(&bytes[4]);
    decoded.angular_velocity_rad_s = DecodeScalar(&bytes[6]);
    command = decoded;
    sequence = bytes[0];
    return true;
}

bool SequenceNewer(uint8_t candidate, uint8_t previous)
{
    const uint8_t distance = static_cast<uint8_t>(candidate - previous);
    return distance != 0U && distance < 128U;
}
}

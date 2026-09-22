/**
 * @file sbus.cpp
 * @brief Streaming S.BUS decoder using the framework UART IDLE+DMA delivery.
 */
#include "sbus.h"

#include "bsp_uart.h"

#include <cstring>

namespace
{
constexpr uint8_t SBUS_HEADER = 0x0FU;
constexpr uint8_t SBUS_FOOTER = 0x00U;
constexpr uint8_t SBUS_FLAG_CHANNEL_17 = 0x01U;
constexpr uint8_t SBUS_FLAG_CHANNEL_18 = 0x02U;
constexpr uint8_t SBUS_FLAG_FRAME_LOST = 0x04U;
constexpr uint8_t SBUS_FLAG_FAILSAFE = 0x08U;

UART_HandleTypeDef *sbus_uart;
uint8_t stream_buffer[SBUS_FRAME_SIZE];
uint8_t stream_length;
Struct_SBUS_Frame latest_frame;
Struct_SBUS_Diagnostics diagnostics;
bool frame_available;

uint32_t EnterCritical()
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    __DMB();
    return primask;
}

void ExitCritical(uint32_t primask)
{
    __DMB();
    __set_PRIMASK(primask);
}

void DecodeFrame(const uint8_t *bytes, Struct_SBUS_Frame *frame)
{
    for (uint32_t channel = 0U; channel < SBUS_CHANNEL_COUNT; ++channel)
    {
        const uint32_t bit_offset = channel * 11U;
        const uint32_t byte_offset = bit_offset / 8U;
        const uint32_t shift = bit_offset % 8U;
        const uint32_t packed = static_cast<uint32_t>(bytes[1U + byte_offset]) |
                                (static_cast<uint32_t>(bytes[2U + byte_offset]) << 8U) |
                                (static_cast<uint32_t>(bytes[3U + byte_offset]) << 16U);
        frame->channels[channel] = static_cast<int16_t>(
            static_cast<int32_t>((packed >> shift) & 0x07FFU) - SBUS_CHANNEL_OFFSET);
    }

    frame->channel_17 = (bytes[23] & SBUS_FLAG_CHANNEL_17) != 0U;
    frame->channel_18 = (bytes[23] & SBUS_FLAG_CHANNEL_18) != 0U;
    frame->frame_lost = (bytes[23] & SBUS_FLAG_FRAME_LOST) != 0U;
    frame->failsafe = (bytes[23] & SBUS_FLAG_FAILSAFE) != 0U;
}

void PublishFrame(const uint8_t *bytes)
{
    Struct_SBUS_Frame decoded{};
    DecodeFrame(bytes, &decoded);
    decoded.timestamp_ms = HAL_GetTick();

    const uint32_t primask = EnterCritical();
    decoded.sequence = latest_frame.sequence + 1U;
    latest_frame = decoded;
    frame_available = true;
    diagnostics.valid_frames++;
    if (decoded.frame_lost != 0U)
    {
        diagnostics.frame_lost_frames++;
    }
    if (decoded.failsafe != 0U)
    {
        diagnostics.failsafe_frames++;
    }
    ExitCritical(primask);
}

void Resynchronize()
{
    uint8_t next_header = SBUS_FRAME_SIZE;
    for (uint8_t index = 1U; index < SBUS_FRAME_SIZE; ++index)
    {
        if (stream_buffer[index] == SBUS_HEADER)
        {
            next_header = index;
            break;
        }
    }

    diagnostics.resync_events++;
    if (next_header == SBUS_FRAME_SIZE)
    {
        stream_length = 0U;
        return;
    }

    stream_length = static_cast<uint8_t>(SBUS_FRAME_SIZE - next_header);
    std::memmove(stream_buffer, &stream_buffer[next_header], stream_length);
}

void ConsumeByte(uint8_t byte)
{
    if (stream_length == 0U)
    {
        if (byte != SBUS_HEADER)
        {
            return;
        }
        stream_buffer[stream_length++] = byte;
        return;
    }

    stream_buffer[stream_length++] = byte;
    if (stream_length != SBUS_FRAME_SIZE)
    {
        return;
    }

    if (stream_buffer[SBUS_FRAME_SIZE - 1U] == SBUS_FOOTER)
    {
        PublishFrame(stream_buffer);
        stream_length = 0U;
        return;
    }

    diagnostics.invalid_candidates++;
    Resynchronize();
}
}

extern "C" bool SBUS_Init(UART_HandleTypeDef *huart)
{
    if (huart == nullptr || huart->hdmarx == nullptr ||
        huart->Init.BaudRate != 100000U ||
        huart->Init.WordLength != UART_WORDLENGTH_9B ||
        huart->Init.StopBits != UART_STOPBITS_2 ||
        huart->Init.Parity != UART_PARITY_EVEN ||
        (huart->Init.Mode & UART_MODE_RX) == 0U)
    {
        return false;
    }

    const uint32_t primask = EnterCritical();
    sbus_uart = huart;
    stream_length = 0U;
    latest_frame = {};
    diagnostics = {};
    frame_available = false;
    ExitCritical(primask);

    UART_Init(huart, SBUS_RxCallback);
    return true;
}

extern "C" void SBUS_RxCallback(uint8_t *buffer, uint16_t length)
{
    if (sbus_uart == nullptr || buffer == nullptr || length == 0U)
    {
        return;
    }

    diagnostics.rx_events++;
    diagnostics.rx_bytes += length;
    diagnostics.last_rx_size = length;
    for (uint16_t index = 0U; index < length; ++index)
    {
        ConsumeByte(buffer[index]);
    }
}

extern "C" bool SBUS_ReadLatest(Struct_SBUS_Frame *frame)
{
    if (frame == nullptr)
    {
        return false;
    }
    const uint32_t primask = EnterCritical();
    const bool available = frame_available;
    if (available)
    {
        *frame = latest_frame;
    }
    ExitCritical(primask);
    return available;
}

extern "C" bool SBUS_IsOnline(void)
{
    const uint32_t primask = EnterCritical();
    const bool available = frame_available;
    const uint32_t timestamp_ms = latest_frame.timestamp_ms;
    ExitCritical(primask);
    return available && (HAL_GetTick() - timestamp_ms <= SBUS_RX_TIMEOUT_MS);
}

extern "C" bool SBUS_IsHealthy(void)
{
    Struct_SBUS_Frame frame{};
    return SBUS_ReadLatest(&frame) && SBUS_IsOnline() &&
           frame.frame_lost == 0U && frame.failsafe == 0U;
}

extern "C" void SBUS_GetDiagnostics(Struct_SBUS_Diagnostics *result)
{
    if (result == nullptr)
    {
        return;
    }
    const uint32_t primask = EnterCritical();
    *result = diagnostics;
    ExitCritical(primask);
}

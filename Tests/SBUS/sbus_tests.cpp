#include "sbus.h"
#include "bsp_uart.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <cstdint>
#include <cstdio>

namespace
{
uint32_t tick_ms;
UART_HandleTypeDef *initialized_uart;
UART_Callback initialized_callback;
DMA_HandleTypeDef rx_dma{};

UART_HandleTypeDef ValidUart()
{
    UART_HandleTypeDef uart{};
    uart.Init.BaudRate = 100000U;
    uart.Init.WordLength = UART_WORDLENGTH_9B;
    uart.Init.StopBits = UART_STOPBITS_2;
    uart.Init.Parity = UART_PARITY_EVEN;
    uart.Init.Mode = UART_MODE_RX;
    uart.hdmarx = &rx_dma;
    return uart;
}

std::array<uint8_t, SBUS_FRAME_SIZE> MakeFrame(
    const std::array<uint16_t, SBUS_CHANNEL_COUNT> &channels,
    uint8_t flags = 0U)
{
    std::array<uint8_t, SBUS_FRAME_SIZE> frame{};
    frame[0] = 0x0FU;
    frame[23] = flags;
    frame[24] = 0x00U;
    for (uint32_t channel = 0; channel < SBUS_CHANNEL_COUNT; ++channel)
    {
        const uint32_t bit_offset = channel * 11U;
        for (uint32_t bit = 0; bit < 11U; ++bit)
        {
            if ((channels[channel] & (1U << bit)) != 0U)
            {
                const uint32_t packed_bit = bit_offset + bit;
                frame[1U + packed_bit / 8U] |= static_cast<uint8_t>(1U << (packed_bit % 8U));
            }
        }
    }
    return frame;
}

void Deliver(uint8_t *data, uint16_t length)
{
    assert(initialized_callback != nullptr);
    initialized_callback(data, length);
}

void TestInvalidConfiguration()
{
    assert(!SBUS_Init(nullptr));
    auto uart = ValidUart();
    uart.hdmarx = nullptr;
    assert(!SBUS_Init(&uart));
    uart = ValidUart(); uart.Init.BaudRate = 115200U; assert(!SBUS_Init(&uart));
    uart = ValidUart(); uart.Init.WordLength = 8U; assert(!SBUS_Init(&uart));
    uart = ValidUart(); uart.Init.StopBits = 1U; assert(!SBUS_Init(&uart));
    uart = ValidUart(); uart.Init.Parity = 0U; assert(!SBUS_Init(&uart));
    uart = ValidUart(); uart.Init.Mode = UART_MODE_TX; assert(!SBUS_Init(&uart));
}

void TestDecodeAndLinkState()
{
    auto uart = ValidUart();
    initialized_uart = nullptr;
    initialized_callback = nullptr;
    tick_ms = 10U;
    assert(SBUS_Init(&uart));
    assert(initialized_uart == &uart);
    assert(SBUS_IsEnabled());
    assert(!SBUS_IsOnline());
    assert(!SBUS_IsDataValid());
    assert(!SBUS_IsHealthy());

    std::array<uint16_t, SBUS_CHANNEL_COUNT> values{};
    for (uint32_t i = 0; i < values.size(); ++i)
    {
        values[i] = static_cast<uint16_t>((i * 137U) & 0x07FFU);
    }
    values[0] = 0U;
    values[1] = 1024U;
    values[2] = 2047U;
    auto frame = MakeFrame(values, 0x03U);
    Deliver(frame.data(), frame.size());

    Struct_SBUS_Frame result{};
    assert(SBUS_ReadLatest(&result));
    for (uint32_t i = 0; i < values.size(); ++i)
    {
        assert(result.channels[i] == static_cast<int16_t>(values[i]) - 1024);
    }
    assert(result.channel_17 == 1U && result.channel_18 == 1U);
    assert(result.sequence == 1U && result.timestamp_ms == 10U);
    assert(SBUS_IsOnline() && SBUS_IsDataValid() && SBUS_IsHealthy());
    tick_ms = 110U;
    assert(SBUS_IsOnline());
    tick_ms = 111U;
    assert(!SBUS_IsOnline() && !SBUS_IsHealthy());
}

void TestStreamingAndRecovery()
{
    auto uart = ValidUart();
    tick_ms = 200U;
    assert(SBUS_Init(&uart));
    std::array<uint16_t, SBUS_CHANNEL_COUNT> six_channels{};
    std::array<uint16_t, SBUS_CHANNEL_COUNT> ten_channels{};
    six_channels.fill(1024U);
    ten_channels.fill(1024U);
    for (uint16_t i = 0U; i < 6U; ++i) six_channels[i] = static_cast<uint16_t>(900U + i);
    for (uint16_t i = 0U; i < 10U; ++i) ten_channels[i] = static_cast<uint16_t>(1100U + i);
    auto first = MakeFrame(six_channels);
    auto second = MakeFrame(ten_channels);

    uint8_t garbage[] = {0x55U, 0xAAU, 0x01U};
    Deliver(garbage, sizeof(garbage));
    Deliver(first.data(), 7U);
    Deliver(first.data() + 7U, 18U);
    Struct_SBUS_Frame result{};
    assert(SBUS_ReadLatest(&result));
    for (uint16_t i = 0U; i < 6U; ++i) assert(result.channels[i] == static_cast<int16_t>(six_channels[i]) - 1024);

    std::array<uint8_t, SBUS_FRAME_SIZE * 2U> joined{};
    std::copy(second.begin(), second.end(), joined.begin());
    std::copy(first.begin(), first.end(), joined.begin() + SBUS_FRAME_SIZE);
    Deliver(joined.data(), joined.size());
    assert(SBUS_ReadLatest(&result));
    assert(result.sequence == 3U);
    assert(result.channels[0] == static_cast<int16_t>(six_channels[0]) - 1024);

    auto broken = first;
    broken[24] = 0x7EU;
    std::array<uint8_t, SBUS_FRAME_SIZE * 2U> recovery{};
    std::copy(broken.begin(), broken.end(), recovery.begin());
    std::copy(second.begin(), second.end(), recovery.begin() + SBUS_FRAME_SIZE);
    Deliver(recovery.data(), recovery.size());
    assert(SBUS_ReadLatest(&result));
    assert(result.sequence == 4U);
    for (uint16_t i = 0U; i < 10U; ++i) assert(result.channels[i] == static_cast<int16_t>(ten_channels[i]) - 1024);

    Struct_SBUS_Diagnostics diagnostics{};
    SBUS_GetDiagnostics(&diagnostics);
    assert(diagnostics.rx_events == 5U);
    assert(diagnostics.rx_bytes == sizeof(garbage) + 7U + 18U + joined.size() + recovery.size());
    assert(diagnostics.valid_frames == 4U);
    assert(diagnostics.invalid_candidates >= 1U);
    assert(diagnostics.resync_events >= 1U);
    assert(diagnostics.last_rx_size == recovery.size());
}

void TestFlagsAndDroppedByteRecovery()
{
    auto uart = ValidUart();
    assert(SBUS_Init(&uart));
    std::array<uint16_t, SBUS_CHANNEL_COUNT> values{};
    values.fill(1024U);
    auto lost = MakeFrame(values, 0x0CU);
    Deliver(lost.data(), lost.size());
    assert(SBUS_IsOnline());
    assert(!SBUS_IsHealthy());

    auto clean = MakeFrame(values);
    std::array<uint8_t, SBUS_FRAME_SIZE * 2U - 1U> stream{};
    std::copy(lost.begin(), lost.begin() + 12, stream.begin());
    std::copy(lost.begin() + 13, lost.end(), stream.begin() + 12);
    std::copy(clean.begin(), clean.end(), stream.begin() + SBUS_FRAME_SIZE - 1U);
    Deliver(stream.data(), stream.size());
    Struct_SBUS_Frame result{};
    assert(SBUS_ReadLatest(&result));
    assert(result.sequence == 2U);
    assert(SBUS_IsHealthy());

    Struct_SBUS_Diagnostics diagnostics{};
    SBUS_GetDiagnostics(&diagnostics);
    assert(diagnostics.frame_lost_frames == 1U);
    assert(diagnostics.failsafe_frames == 1U);
}
}

extern "C" uint32_t HAL_GetTick(void) { return tick_ms; }
extern "C" uint32_t __get_PRIMASK(void) { return 0U; }
extern "C" void __disable_irq(void) {}
extern "C" void __set_PRIMASK(uint32_t) {}
extern "C" void __DMB(void) {}

void UART_Init(UART_HandleTypeDef *huart, UART_Callback callback)
{
    initialized_uart = huart;
    initialized_callback = callback;
}

int main()
{
    TestInvalidConfiguration();
    TestDecodeAndLinkState();
    TestStreamingAndRecovery();
    TestFlagsAndDroppedByteRecovery();
    std::puts("S.BUS tests passed");
    return 0;
}

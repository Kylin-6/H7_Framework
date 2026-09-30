#include "board_transport.h"

#include "bsp_can.h"
#include "daemon.h"
#include "message_center.h"
#include "stm32h7xx.h"
#include "sys_timestamp.h"
#include "transport_config.h"
#include "transport_protocol.h"

namespace
{
// FDCAN RX ISR 写最新帧和实际接收时间；ControlTask 在 PRIMASK 临界区复制后解码。
volatile uint8_t pending_bytes[TransportProtocol::kPayloadSize];
volatile uint64_t pending_rx_us;
volatile bool pending;
bool initialized;
bool has_sequence;
uint8_t last_sequence;
uint64_t last_valid_rx_us;
Daemon transport_daemon{TransportProtocol::kCommandMaxAgeUs / 1000U};

void Receive(FDCAN_HandleTypeDef *bus, uint32_t id, uint8_t *data,
             uint32_t size, void *)
{
    if (bus != TransportConfig_Bus() ||
        id != TransportConfig::kChassisFeedbackCanId ||
        data == nullptr || size != TransportProtocol::kPayloadSize)
    {
        return;
    }
    for (uint8_t index = 0U; index < TransportProtocol::kPayloadSize; ++index)
    {
        pending_bytes[index] = data[index];
    }
    pending_rx_us = SYS_Timestamp_Get_Microsecond();
    pending = true;
}
}

FDCAN_HandleTypeDef *TransportConfig_Bus(void)
{
    return &hfdcan2;
}

bool BoardTransport_Init(void)
{
    if (!initialized)
    {
        initialized = BSP_CAN_RegisterCallback(TransportConfig::kChassisFeedbackCanId,
            TransportConfig_Bus(), Receive, nullptr) &&
            DaemonManager::Register(transport_daemon);
    }
    return initialized;
}

bool BoardTransport_IsOnline(void)
{
    return initialized && transport_daemon.IsOnline();
}

uint32_t BoardTransport_OfflineDurationMs(void)
{
    return transport_daemon.OfflineDurationMs();
}

void BoardTransport_Poll(void)
{
    if (!initialized) { return; }
    uint8_t bytes[TransportProtocol::kPayloadSize];
    uint64_t received_us = 0U;
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const bool available = pending;
    if (available)
    {
        for (uint8_t index = 0U; index < TransportProtocol::kPayloadSize; ++index)
        {
            bytes[index] = pending_bytes[index];
        }
        received_us = pending_rx_us;
        pending = false;
    }
    __DMB();
    __set_PRIMASK(primask);
    if (!available)
    {
        return;
    }
    const uint64_t now_us = SYS_Timestamp_Get_Microsecond();
    if (now_us < received_us ||
        now_us - received_us > TransportProtocol::kCommandMaxAgeUs)
    {
        return;
    }
    ChassisFeedback feedback{};
    uint8_t sequence = 0U;
    if (!TransportProtocol::DecodeChassisFeedback(bytes, sizeof(bytes),
                                                  feedback, sequence))
    {
        return;
    }
    // 合法重复帧只证明链路活性；连续数据流不中断时不能重建基准并刷新旧 Topic。
    transport_daemon.Feed();
    if (has_sequence && received_us - last_valid_rx_us >
                            TransportProtocol::kCommandMaxAgeUs)
    {
        has_sequence = false;
    }
    last_valid_rx_us = received_us;
    if (has_sequence && !TransportProtocol::SequenceNewer(sequence, last_sequence))
    {
        return;
    }
    MessageCenter::Chassis_Feedback_Topic.PublishAt(feedback, received_us);
    last_sequence = sequence;
    has_sequence = true;
}

void BoardTransport_SendChassis(const ChassisCmd &command)
{
    if (!initialized) { return; }
    static uint8_t sequence;
    Struct_CAN_Tx_Msg message{};
    message.hfdcan = TransportConfig_Bus();
    message.id = TransportConfig::kChassisCmdCanId;
    message.len = TransportProtocol::kPayloadSize;
    if (!TransportProtocol::EncodeChassisCmd(command, sequence, message.data))
    {
        return;
    }
    if (CAN_Tx_Perform(&message))
    {
        ++sequence;
    }
}

#include "board_transport.h"

#include "bsp_can.h"
#include "message_center.h"
#include "stm32h7xx.h"
#include "sys_timestamp.h"
#include "transport_config.h"
#include "transport_protocol.h"

namespace
{
// FDCAN RX ISR 写最新帧和实际接收时间；ControlTask 在 PRIMASK 临界区复制后解码。
volatile uint8_t pending_bytes[TransportProtocol::kPayloadSize];
volatile bool pending;
volatile uint64_t pending_rx_us;
bool initialized;
bool has_sequence;
uint8_t last_sequence;
uint64_t last_accepted_rx_us;
Subscriber<ChassisFeedback> feedback_subscriber(MessageCenter::Chassis_Feedback_Topic);
uint8_t feedback_sequence;

void Receive(FDCAN_HandleTypeDef *bus, uint32_t id, uint8_t *data,
             uint32_t size, void *)
{
    if (bus != TransportConfig_Bus() ||
        id != TransportConfig::kChassisCmdCanId ||
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

void SendFeedback(void)
{
    ChassisFeedback feedback{};
    if (!feedback_subscriber.Read(feedback))
    {
        return;
    }
    Struct_CAN_Tx_Msg message{};
    message.hfdcan = TransportConfig_Bus();
    message.id = TransportConfig::kChassisFeedbackCanId;
    message.len = TransportProtocol::kPayloadSize;
    if (TransportProtocol::EncodeChassisFeedback(feedback, feedback_sequence,
                                                  message.data) &&
        CAN_Tx_Perform(&message))
    {
        ++feedback_sequence;
    }
}

void ProcessCommand(const uint8_t *bytes, uint64_t received_us)
{
    const uint64_t now_us = SYS_Timestamp_Get_Microsecond();
    if (now_us < received_us ||
        now_us - received_us > TransportProtocol::kCommandMaxAgeUs)
    {
        return;
    }

    ChassisCmd command{};
    uint8_t sequence = 0U;
    if (!TransportProtocol::DecodeChassisCmd(bytes, TransportProtocol::kPayloadSize,
                                             command, sequence))
    {
        return;
    }
    // 超时重建只看已通过解码/当前时效检查的实际 RX 时间；无会话标识，旧合法帧仍可能被重建接受。
    if (has_sequence && received_us - last_accepted_rx_us >
                            TransportProtocol::kCommandMaxAgeUs)
    {
        has_sequence = false;
    }
    if (has_sequence && !TransportProtocol::SequenceNewer(sequence, last_sequence))
    {
        return;
    }
    MessageCenter::Chassis_Command_Topic.PublishAt(command, received_us);
    last_sequence = sequence;
    last_accepted_rx_us = received_us;
    has_sequence = true;
}
}

FDCAN_HandleTypeDef *TransportConfig_Bus(void)
{
    return &hfdcan3;
}

void BoardTransport_Init(void)
{
    if (!initialized)
    {
        initialized = BSP_CAN_RegisterCallback(TransportConfig::kChassisCmdCanId,
            TransportConfig_Bus(), Receive, nullptr);
    }
}

void BoardTransport_Poll(void)
{
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
    if (available)
    {
        ProcessCommand(bytes, received_us);
    }
    SendFeedback();
}

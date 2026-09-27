#include "board_transport.h"

#include "bsp_can.h"
#include "transport_config.h"
#include "transport_protocol.h"

FDCAN_HandleTypeDef *TransportConfig_Bus(void)
{
    return &hfdcan2;
}

void BoardTransport_Init(void) {}
void BoardTransport_Poll(void) {}

void BoardTransport_SendChassis(const ChassisCmd &command)
{
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

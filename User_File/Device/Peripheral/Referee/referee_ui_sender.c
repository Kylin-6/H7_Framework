#include "referee_ui_sender.h"

#include "crc_ref.h"
#include "referee_26.h"

#include <string.h>

_Static_assert(sizeof(interaction_figure_t) == 15U, "UI graphic wire size");
_Static_assert(sizeof(ext_client_custom_character_t) == 45U, "UI text wire size");

uint8_t RefereeUIRobotIdIsValid(uint16_t robot_id)
{
    return (robot_id >= 3U && robot_id <= 5U) || (robot_id >= 103U && robot_id <= 105U);
}

static uint8_t TrySendPacket(uint16_t robot_id, uint16_t content_id, const void* content, uint16_t length)
{
    static uint8_t sequence;
    uint8_t packet[120] = {0};
    if (!RefereeUIRobotIdIsValid(robot_id))
        return FALSE;
    const uint16_t payload_length = 6U + length;
    const uint16_t client_id = 0x100U + robot_id;
    packet[0] = REFEREE_SOF;
    packet[1] = (uint8_t) payload_length;
    packet[2] = (uint8_t) (payload_length >> 8U);
    packet[3] = sequence;
    Append_CRC8_Check_Sum(packet, 5U);
    packet[5] = 0x01U;
    packet[6] = 0x03U;
    packet[7] = (uint8_t) content_id;
    packet[8] = (uint8_t) (content_id >> 8U);
    packet[9] = (uint8_t) robot_id;
    packet[10] = (uint8_t) (robot_id >> 8U);
    packet[11] = (uint8_t) client_id;
    packet[12] = (uint8_t) (client_id >> 8U);
    memcpy(packet + 13U, content, length);
    Append_CRC16_Check_Sum(packet, payload_length + 9U);
    if (!RefereeTrySend(packet, payload_length + 9U))
        return FALSE;
    ++sequence;
    return TRUE;
}

uint8_t RefereeUITryDraw(uint16_t robot_id, const interaction_figure_t* figures, uint8_t count)
{
    uint16_t content_id;
    switch (count)
    {
    case 1U:
        content_id = 0x0101U;
        break;
    case 2U:
        content_id = 0x0102U;
        break;
    case 5U:
        content_id = 0x0103U;
        break;
    case 7U:
        content_id = 0x0104U;
        break;
    default:
        return FALSE;
    }
    return figures != NULL && TrySendPacket(robot_id, content_id, figures, count * sizeof(*figures));
}

uint8_t RefereeUITryText(uint16_t robot_id, const ext_client_custom_character_t* text)
{
    return text != NULL && TrySendPacket(robot_id, 0x0110U, text, sizeof(*text));
}

uint8_t RefereeUITryDeleteLayer(uint16_t robot_id, uint8_t layer)
{
    const uint8_t deletion[2] = {1U, layer};
    return layer <= 9U && TrySendPacket(robot_id, 0x0100U, deletion, sizeof(deletion));
}

#include "ClientUI.h"

#include "ClientUI_Layout.h"
#include "message_center.h"
#include "referee_ui_sender.h"

#include <cstring>

namespace
{
bool initialized;
uint16_t configured_robot_id;
uint16_t client_robot_id;
uint32_t last_send_ms;
uint32_t rebuild_ms;
uint8_t rebuild_slot;
uint8_t dynamic_visible;
uint8_t text_pending;
bool rebuilding;
bool draw_dynamic;
bool readd_dynamic;
char sent_text[4][30];
bool sent_warning[3];

bool DrawStatic(uint16_t robot_id)
{
    using namespace ClientUILayout;
    if (rebuild_slot < kStaticGroups)
    {
        const uint8_t count = rebuild_slot == kStaticGroups - 1U ? 2U : 7U;
        return RefereeUITryDraw(robot_id, kStaticFigures + rebuild_slot * 7U, count);
    }
    const uint8_t index = rebuild_slot - kStaticGroups;
    const char* labels[kTextCount] = {"F", "E", "A   B   C", "CAP N/A", "kRPM", "LOAD"};
    const uint16_t x[kTextCount] = {697U, 605U, 912U, 440U, 1670U, 1535U};
    const uint16_t y[kTextCount] = {775U, 550U, 275U, 700U, 730U, 614U};
    ext_client_custom_character_t text{};
    text.graphic_data_struct = Figure(150U + index, figure_type_Char, color_White, 1U, x[index], y[index]);
    text.graphic_data_struct.details_a = index < 3U ? 20U : 14U;
    const size_t length = std::strlen(labels[index]);
    text.graphic_data_struct.details_b = length;
    std::memcpy(text.data, labels[index], length);
    return RefereeUITryText(robot_id, &text);
}

bool DrawDynamic(uint16_t robot_id, const Struct_Client_UI_State& view)
{
    interaction_figure_t graphs[7]{};
    ClientUILayout::BuildDynamic(view, graphs);
    uint8_t next_visible = 0U;
    for (uint8_t i = 0U; i < 7U; ++i)
    {
        if (graphs[i].operate_type == operate_type_Del)
            continue;
        const uint8_t bit = 1U << i;
        next_visible |= bit;
        if (!readd_dynamic && (dynamic_visible & bit) != 0U)
            graphs[i].operate_type = operate_type_Change;
    }
    if (!RefereeUITryDraw(robot_id, graphs, 7U))
        return false;
    dynamic_visible = next_visible;
    readd_dynamic = false;
    return true;
}

bool DrawText(uint16_t robot_id, uint8_t index, const char* label, bool warning)
{
    ext_client_custom_character_t text{};
    const bool mode = index == 3U;
    text.graphic_data_struct = ClientUILayout::Figure(170U + index, figure_type_Char,
                                                      mode ? color_White : warning ? color_Yellow
                                                                                   : color_Orange,
                                                      2U,
                                                      mode ? 1410U : 60U, mode ? 560U : 800U - 30U * index);
    text.graphic_data_struct.details_a = 18U;
    text.graphic_data_struct.operate_type = *label == '\0'                                                        ? operate_type_Del
                                            : sent_text[index][0] == '\0' || (text_pending & (1U << index)) != 0U ? operate_type_ADD
                                                                                                                  : operate_type_Change;
    const size_t length = std::strlen(label);
    text.graphic_data_struct.details_b = length;
    std::memcpy(text.data, label, length);
    return RefereeUITryText(robot_id, &text);
}

bool Expired(uint64_t timestamp_us, uint64_t now_us)
{
    return now_us < timestamp_us || now_us - timestamp_us > ClientUIConfig::kFeedbackMaxAgeUs;
}
} // namespace

bool ClientUI_Init(uint16_t robot_id)
{
    // 仅由 StatusTask 初始化和消费私有刷新状态，不与 Capture 共享可变缓存。
    initialized = RefereeUIIsReady() && (robot_id == 0U || RefereeUIRobotIdIsValid(robot_id));
    configured_robot_id = robot_id;
    client_robot_id = 0U;
    last_send_ms = static_cast<uint32_t>(SYS_Timestamp_Get_Microsecond() / 1000U);
    return initialized;
}

void ClientUI_Update(void)
{
    if (!initialized)
        return;
    uint16_t robot_id = configured_robot_id;
    if (robot_id == 0U && !RefereeReadUIRobotId(&robot_id))
    {
        // 身份恢复后即使 ID 相同，也重新建立本 UI 图层。
        client_robot_id = 0U;
        return;
    }
    const uint64_t now_us = SYS_Timestamp_Get_Microsecond();
    const uint32_t now_ms = static_cast<uint32_t>(now_us / 1000U);
    if (now_ms - last_send_ms < ClientUIConfig::kSendIntervalMs)
        return;
    if (robot_id != client_robot_id)
    {
        if (!RefereeUITryDeleteLayer(robot_id, 9U))
            return;
        client_robot_id = robot_id;
        rebuild_slot = dynamic_visible = text_pending = 0U;
        rebuilding = draw_dynamic = readd_dynamic = true;
        std::memset(sent_text, 0, sizeof(sent_text));
        std::memset(sent_warning, 0, sizeof(sent_warning));
        rebuild_ms = last_send_ms = now_ms;
        return;
    }
    if (!rebuilding && now_ms - rebuild_ms >= ClientUIConfig::kRebuildIntervalMs)
    {
        rebuilding = true;
        rebuild_slot = 0U;
        readd_dynamic = true;
        text_pending = 0x0FU;
        rebuild_ms = now_ms;
    }
    Struct_Client_UI_State view{};
    (void) MessageCenter::Client_UI_State_Topic.ReadFresh(view, ClientUIConfig::kSnapshotMaxAgeUs);
    // 保留源时间：即使采集刚停止，也不额外延长 100 ms 的反馈有效期。
    if (Expired(view.pitch_timestamp_us, now_us))
        view.pitch_valid = false;
    if (Expired(view.shoot_timestamp_us, now_us))
    {
        view.speed_valid = false;
        view.friction = view.loader = Enum_Client_UI_Motion::Unknown;
    }
    if (Expired(view.chassis_timestamp_us, now_us))
    {
        view.chassis = Enum_Client_UI_Chassis::Unknown;
        std::strcpy(view.mode_text, "UNKNOWN");
    }
    view.mode_text[29] = '\0';
    for (auto& line : view.alert_text)
        line[29] = '\0';

    bool submitted = false;
    bool auxiliary = false;
    if (!draw_dynamic)
    {
        // 报警、模式文字、静态图元依次优先；动态包始终与辅助包交替。
        for (uint8_t i = 0U; i < 4U; ++i)
        {
            const char* label = i == 3U ? view.mode_text : view.alert_text[i];
            const bool warning = i < 3U && view.alert_warning[i];
            if ((text_pending & (1U << i)) == 0U && std::strcmp(label, sent_text[i]) == 0 &&
                (i == 3U || warning == sent_warning[i]))
                continue;
            auxiliary = true;
            submitted = DrawText(robot_id, i, label, warning);
            if (submitted)
            {
                std::memcpy(sent_text[i], label, sizeof(sent_text[i]));
                if (i < 3U)
                    sent_warning[i] = warning;
                text_pending &= ~(1U << i);
            }
            break;
        }
        if (!auxiliary && rebuilding)
        {
            auxiliary = true;
            submitted = DrawStatic(robot_id);
            if (submitted && ++rebuild_slot == ClientUILayout::kStaticGroups + ClientUILayout::kTextCount)
                rebuilding = false;
        }
    }
    if (!auxiliary)
        submitted = DrawDynamic(robot_id, view);
    if (!submitted)
        return;
    last_send_ms = now_ms;
    draw_dynamic = auxiliary;
}

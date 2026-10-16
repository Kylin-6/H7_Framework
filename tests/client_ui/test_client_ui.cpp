#include "ClientUI.h"
#include "ClientUI_Layout.h"
#include "bsp_uart.h"
#include "message_center.h"
#include "referee_26.h"
#include "referee_daemon.h"
#include "referee_ui_sender.h"
extern "C"
{
#include "crc_ref.h"
}

#include <array>
#include <cassert>
#include <cstdio>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

uint32_t test_primask;
static uint64_t now_us;
static uint8_t tx_status = HAL_OK;
static bool register_ok = true;
static unsigned init_count;
static unsigned feeds;
static UART_HandleTypeDef uart{&init_count, &init_count};
static void (*rx_callback)(uint8_t*, uint16_t);
struct Packet
{
    uint32_t ms;
    uint8_t status;
    std::vector<uint8_t> bytes;
};
static std::vector<Packet> packets;
extern "C" uint64_t SYS_Timestamp_Get_Microsecond(void)
{
    return now_us;
}
extern "C" uint32_t HAL_GetTick(void)
{
    return static_cast<uint32_t>(now_us / 1000U);
}
extern "C" void UART_Init(UART_HandleTypeDef* handle, void (*callback)(uint8_t*, uint16_t))
{
    assert(handle == &uart);
    rx_callback = callback;
    ++init_count;
}
extern "C" uint8_t UART_Transmit_Data(UART_HandleTypeDef* handle, uint8_t* data, uint16_t size)
{
    assert(handle == &uart && handle->hdmatx != nullptr);
    packets.push_back({HAL_GetTick(), tx_status, {data, data + size}});
    return tx_status;
}
extern "C" void osDelay(uint32_t)
{
    assert(false && "UI must never block");
}
extern "C" bool RefereeDaemonRegister(void)
{
    return register_ok;
}
extern "C" void RefereeDaemonFeed(void)
{
    ++feeds;
}
extern "C" bool RefereeDaemonIsOnline(void)
{
    return feeds != 0U;
}

static uint16_t U16(const std::vector<uint8_t>& data, size_t offset)
{
    return data[offset] | (uint16_t(data[offset + 1U]) << 8U);
}
// Independent bitwise CRC oracle, rather than comparing the encoder with itself.
static uint16_t CRC(const uint8_t* data, size_t length, uint16_t initial, uint16_t polynomial)
{
    uint16_t crc = initial;
    for (size_t i = 0; i < length; ++i)
    {
        crc ^= data[i];
        for (unsigned bit = 0; bit < 8U; ++bit)
            crc = (crc >> 1U) ^ ((crc & 1U) ? polynomial : 0U);
    }
    return crc;
}
static void CheckPacket(const Packet& packet, uint16_t robot, uint16_t content, size_t length)
{
    const auto& data = packet.bytes;
    assert(data.size() == length + 15U && data[0] == 0xA5U);
    assert(U16(data, 1U) == length + 6U && U16(data, 5U) == 0x0301U);
    assert(U16(data, 7U) == content && U16(data, 9U) == robot && U16(data, 11U) == robot + 0x100U);
    assert(data[4] == CRC(data.data(), 4U, 0xFFU, 0x8CU));
    assert(U16(data, data.size() - 2U) == CRC(data.data(), data.size() - 2U, 0xFFFFU, 0x8408U));
}
static std::vector<uint8_t> Frame(uint16_t command, uint16_t length, uint8_t id)
{
    std::vector<uint8_t> frame(length + 9U);
    frame[0] = 0xA5U;
    frame[1] = length;
    frame[2] = length >> 8U;
    frame[5] = command;
    frame[6] = command >> 8U;
    frame[7] = id;
    Append_CRC8_Check_Sum(frame.data(), 5U);
    Append_CRC16_Check_Sum(frame.data(), frame.size());
    return frame;
}
static void ReceiveId(uint8_t id)
{
    auto frame = Frame(0x0201U, 13U, id);
    rx_callback(frame.data(), frame.size());
}
static void Start(uint16_t id = 3U)
{
    assert(RefereeInit(&uart) != nullptr);
    assert(ClientUI_Init(id));
}
static bool Dynamic(const Packet& p)
{
    return U16(p.bytes, 7U) == 0x0104U && p.bytes[15] == 100U;
}
static interaction_figure_t FigureAt(const Packet& p, unsigned index)
{
    interaction_figure_t figure;
    std::memcpy(&figure, p.bytes.data() + 13U + 15U * index, sizeof(figure));
    return figure;
}
static Struct_Client_UI_State LiveState()
{
    Struct_Client_UI_State state{};
    state.pitch_valid = state.speed_valid = true;
    state.pitch_timestamp_us = state.shoot_timestamp_us = state.chassis_timestamp_us = now_us;
    state.pitch_rad = 0.2f;
    state.left_speed_rad_s = 100.0f;
    state.right_speed_rad_s = 200.0f;
    state.friction = Enum_Client_UI_Motion::Moving;
    state.loader = Enum_Client_UI_Motion::Stopped;
    state.chassis = Enum_Client_UI_Chassis::Spin;
    std::strcpy(state.mode_text, "SPIN");
    return state;
}
static void Tick(uint64_t milliseconds, bool live = true)
{
    now_us = milliseconds * 1000U;
    if (live)
        MessageCenter::Client_UI_State_Topic.Publish(LiveState());
    const size_t before = packets.size();
    ClientUI_Update();
    assert(packets.size() - before <= 1U);
}

static void Protocol()
{
    assert(!RefereeUITryDeleteLayer(3U, 9U));
    assert(packets.empty());
    Start();
    interaction_figure_t graphs[7]{};
    graphs[0] = ClientUILayout::Line(1U, color_White, 2U, 10U, 20U, 30U, 40U);
    const uint8_t counts[] = {1U, 2U, 5U, 7U};
    for (unsigned i = 0; i < 4U; ++i)
    {
        assert(RefereeUITryDraw(103U, graphs, counts[i]));
        CheckPacket(packets.back(), 103U, 0x0101U + i, counts[i] * 15U);
        assert(packets.back().bytes[3] == i);
    }
    // Little-endian packed graphic bit layout: name + word0 + word1 + word2.
    const auto& bytes = packets.front().bytes;
    assert(bytes[13] == 'H' && bytes[14] == 'U' && bytes[15] == 1U);
    const uint32_t expected[3] = {1U | (9U << 6U) | (8U << 10U), 2U | (10U << 10U) | (20U << 21U), (30U << 10U) | (40U << 21U)};
    for (unsigned word = 0; word < 3U; ++word)
        for (unsigned byte = 0; byte < 4U; ++byte)
            assert(bytes[16U + 4U * word + byte] == ((expected[word] >> (8U * byte)) & 0xFFU));
    for (uint8_t status : {HAL_BUSY, HAL_ERROR})
    {
        tx_status = status;
        assert(!RefereeUITryDeleteLayer(3U, 9U));
        const auto failed = packets.back().bytes;
        tx_status = HAL_OK;
        assert(RefereeUITryDeleteLayer(3U, 9U));
        assert(failed == packets.back().bytes);
    }
    ext_client_custom_character_t text{};
    assert(RefereeUITryText(105U, &text));
    CheckPacket(packets.back(), 105U, 0x0110U, 45U);
    size_t before = packets.size();
    assert(!RefereeUITryDraw(3U, graphs, 3U));
    assert(!RefereeUITryDraw(3U, nullptr, 1U));
    assert(!RefereeUITryText(3U, nullptr));
    assert(!RefereeUITryDeleteLayer(3U, 10U));
    assert(!RefereeUITryDeleteLayer(0U, 9U));
    assert(!RefereeUITryDeleteLayer(7U, 9U));
    assert(!RefereeTrySend(nullptr, 1U));
    uart.hdmatx = nullptr;
    assert(!RefereeUITryDeleteLayer(3U, 9U) && !ClientUI_Init(3U));
    assert(packets.size() == before);
    uart.hdmatx = &init_count;
    assert(!ClientUI_Init(101U));
    register_ok = false;
    assert(RefereeInit(&uart) == nullptr && !ClientUI_Init());
    // Failed initialization stays harmless.
    Tick(1000U);
    assert(packets.size() == before);
}

static void Identity()
{
    Start(0U);
    uint16_t id = 999U;
    assert(!RefereeReadUIRobotId(&id) && id == 999U);
    auto bad = Frame(0x0201U, 13U, 3U);
    bad.back() ^= 1U;
    RefereeReceiveData(bad.data(), bad.size());
    auto short_frame = Frame(0x0201U, 12U, 3U);
    RefereeReceiveData(short_frame.data(), short_frame.size());
    assert(!RefereeReadUIRobotId(&id));
    auto good = Frame(0x0201U, 13U, 104U);
    RefereeReceiveData(good.data(), 4U);
    assert(!RefereeReadUIRobotId(&id));
    RefereeReceiveData(good.data() + 4U, good.size() - 4U);
    test_primask = 1U;
    assert(RefereeReadUIRobotId(&id) && id == 104U && test_primask == 1U);
    test_primask = 0U;
    now_us = 499000U;
    assert(RefereeReadUIRobotId(&id));
    auto unrelated = Frame(0x7777U, 1U, 3U);
    RefereeReceiveData(unrelated.data(), unrelated.size());
    now_us = 500000U;
    assert(!RefereeReadUIRobotId(&id) && id == 104U);
    for (uint8_t valid : {3U, 4U, 5U, 103U, 104U, 105U})
    {
        ReceiveId(valid);
        assert(RefereeReadUIRobotId(&id) && id == valid);
    }
    ReceiveId(7U);
    assert(!RefereeReadUIRobotId(&id));
    now_us = (uint64_t(UINT32_MAX) - 20U) * 1000U;
    ReceiveId(3U);
    now_us += 499000U;
    assert(RefereeReadUIRobotId(&id));
    now_us += 1000U;
    assert(!RefereeReadUIRobotId(&id));
}

static void Capture()
{
    using namespace MessageCenter;
    ClientUI_Capture();
    auto view = Client_UI_State_Topic.ReadWithMeta();
    assert(view.valid && !view.data.pitch_valid && !view.data.speed_valid);
    assert(view.data.autoaim == Enum_Client_UI_AutoAim::Unknown && !view.data.yaw_valid);
    const auto sequence = view.sequence;
    now_us = 9999U;
    ClientUI_Capture();
    assert(Client_UI_State_Topic.Sequence() == sequence);
    now_us = 10000U;
    GimbalFeedback gimbal{};
    gimbal.pitch_rad = 0.5f;
    gimbal.yaw_rad = 1.5f;
    gimbal.ins_valid = true;
    ShootFeedback shoot{};
    shoot.online = true;
    shoot.friction_left_speed_rad_s = -100.0f;
    shoot.friction_right_speed_rad_s = 200.0f;
    shoot.loader_speed_rad_s = -2.0f;
    Gimbal_Feedback_Topic.Publish(gimbal);
    Shoot_Feedback_Topic.Publish(shoot);
    ChassisFeedback chassis{};
    chassis.online = chassis.enabled = true;
    Chassis_Feedback_Topic.Publish(chassis);
    ChassisCmd command{};
    command.mode = ChassisMode::ROTATE;
    Chassis_Command_Topic.Publish(command);
    ClientUI_Capture();
    view = Client_UI_State_Topic.ReadWithMeta();
    assert(view.data.pitch_valid && view.data.pitch_rad == 0.5f && !view.data.yaw_valid);
    assert(view.data.speed_valid && view.data.left_speed_rad_s == 100.0f);
    assert(view.data.loader == Enum_Client_UI_Motion::Moving);
    assert(view.data.chassis == Enum_Client_UI_Chassis::Spin && std::strcmp(view.data.mode_text, "SPIN") == 0);
    now_us = 110000U;
    ClientUI_Capture();
    view = Client_UI_State_Topic.ReadWithMeta();
    assert(view.data.pitch_valid && view.data.pitch_timestamp_us == 10000U);
    now_us = 120000U;
    ClientUI_Capture();
    view = Client_UI_State_Topic.ReadWithMeta();
    assert(!view.data.pitch_valid && !view.data.speed_valid && view.data.chassis == Enum_Client_UI_Chassis::Unknown);
    for (unsigned step = 0U; step < 5U; ++step)
    {
        now_us += 10000U;
        gimbal.pitch_rad = step == 0U ? std::numeric_limits<float>::quiet_NaN() : 0.3f;
        gimbal.ins_valid = step != 1U;
        shoot.friction_left_speed_rad_s = std::numeric_limits<float>::infinity();
        shoot.loader_speed_rad_s = 0.0f;
        shoot.online = step != 1U;
        Gimbal_Feedback_Topic.Publish(gimbal);
        Shoot_Feedback_Topic.Publish(shoot);
        chassis.enabled = step == 4U;
        chassis.online = step != 0U;
        command.mode = step == 2U ? ChassisMode::ZERO_FORCE : ChassisMode::NO_FOLLOW;
        Chassis_Feedback_Topic.Publish(chassis);
        Chassis_Command_Topic.Publish(command);
        ClientUI_Capture();
        const auto state = Client_UI_State_Topic.ReadWithMeta().data;
        assert(state.pitch_valid == (step >= 2U) && !state.speed_valid);
        assert(state.loader == (step == 1U ? Enum_Client_UI_Motion::Unknown : Enum_Client_UI_Motion::Stopped));
        const Enum_Client_UI_Chassis expected[] = {Enum_Client_UI_Chassis::Unknown, Enum_Client_UI_Chassis::Waiting,
                                                   Enum_Client_UI_Chassis::Stopped, Enum_Client_UI_Chassis::Waiting, Enum_Client_UI_Chassis::Active};
        assert(state.chassis == expected[step]);
    }
}

static void Layout()
{
    interaction_figure_t graphs[7]{}, expected[7]{};
    auto state = LiveState();
    state.pitch_rad = 40.0f / ClientUILayout::kRadToDeg;
    state.left_speed_rad_s = state.right_speed_rad_s = 7000.0f / ClientUILayout::kRadSToRpm;
    ClientUILayout::BuildDynamic(state, expected);
    state.pitch_rad = 100.0f;
    state.left_speed_rad_s = state.right_speed_rad_s = 100000.0f;
    ClientUILayout::BuildDynamic(state, graphs);
    assert(std::memcmp(&graphs[1], &expected[1], sizeof(graphs[1])) == 0);
    assert(graphs[4].start_y == 777U && graphs[5].start_y == 777U);
    assert(graphs[3].details_a == 0U && graphs[3].details_b == 360U);
    assert(graphs[2].operate_type == operate_type_Del && graphs[0].operate_type == operate_type_Del);
    state.pitch_rad = std::numeric_limits<float>::quiet_NaN();
    state.left_speed_rad_s = -1.0f;
    state.right_speed_rad_s = std::numeric_limits<float>::infinity();
    state.yaw_valid = true;
    state.relative_yaw_rad = std::numeric_limits<float>::max();
    ClientUILayout::BuildDynamic(state, graphs);
    assert(graphs[1].operate_type == operate_type_Del && graphs[4].operate_type == operate_type_Del && graphs[5].operate_type == operate_type_Del);
    assert(graphs[3].details_a < 360U && graphs[3].details_b < 360U);
}

static void Schedule()
{
    Start();
    for (unsigned ms = 0; ms < 100U; ms += 10U)
        Tick(ms);
    assert(packets.empty());
    tx_status = HAL_BUSY;
    Tick(100U);
    const auto failed = packets.back().bytes;
    tx_status = HAL_OK;
    Tick(110U);
    assert(packets.back().bytes == failed);
    Tick(209U);
    assert(packets.size() == 2U);
    Tick(210U);
    assert(Dynamic(packets.back()) && FigureAt(packets.back(), 1U).operate_type == operate_type_ADD);
    // On a failed dynamic update the same ADD/Change state and sequence must be retried.
    Tick(310U);
    tx_status = HAL_ERROR;
    Tick(410U);
    const auto dynamic_failed = packets.back().bytes;
    tx_status = HAL_OK;
    Tick(420U);
    assert(packets.back().bytes == dynamic_failed && FigureAt(packets.back(), 1U).operate_type == operate_type_Change);
    for (unsigned ms = 430U; ms <= 9000U; ms += 10U)
        Tick(ms);
    uint32_t last_ms = 0U;
    bool sent = false, readded = false;
    unsigned clears = 0U;
    std::array<bool, 37> static_seen{};
    std::array<bool, 6> labels_seen{};
    for (const auto& packet : packets)
    {
        if (packet.status != HAL_OK)
            continue;
        assert(!sent || packet.ms - last_ms >= 100U);
        sent = true;
        last_ms = packet.ms;
        const auto content = U16(packet.bytes, 7U);
        if (content == 0x0100U)
            ++clears;
        else if (Dynamic(packet))
            readded |= packet.ms >= 5110U && FigureAt(packet, 1U).operate_type == operate_type_ADD;
        else if (content == 0x0104U || content == 0x0102U)
        {
            const unsigned count = content == 0x0104U ? 7U : 2U;
            for (unsigned i = 0; i < count; ++i)
            {
                const auto figure = FigureAt(packet, i);
                assert(figure.layer == 9U && figure.figure_name[2] < static_seen.size());
                static_seen[figure.figure_name[2]] = true;
            }
        }
        else if (content == 0x0110U)
        {
            const uint8_t id = packet.bytes[15];
            if (id >= 150U && id < 156U)
                labels_seen[id - 150U] = true;
        }
    }
    assert(clears == 1U && readded);
    for (bool seen : static_seen)
        assert(seen);
    for (bool seen : labels_seen)
        assert(seen);
}

static void Expiry()
{
    Start();
    Tick(100U);
    Tick(200U);
    assert(FigureAt(packets.back(), 1U).operate_type == operate_type_ADD);
    // Snapshot still <200 ms old, but its feedback has aged past 100 ms.
    Tick(300U, false);
    Tick(400U, false);
    assert(Dynamic(packets.back()));
    assert(FigureAt(packets.back(), 1U).operate_type == operate_type_Del);
    assert(FigureAt(packets.back(), 4U).operate_type == operate_type_Del);
    assert(FigureAt(packets.back(), 6U).color == color_White);
    Tick(500U);
    Tick(600U);
    assert(Dynamic(packets.back()) && FigureAt(packets.back(), 1U).operate_type == operate_type_ADD);
    // Explicit extension inputs disappear when their overall snapshot expires.
    now_us = 700000U;
    auto view = LiveState();
    view.yaw_valid = true;
    view.relative_yaw_rad = 0.3f;
    view.autoaim = Enum_Client_UI_AutoAim::Active;
    std::memset(view.alert_text[0], 'A', sizeof(view.alert_text[0]));
    MessageCenter::Client_UI_State_Topic.Publish(view);
    ClientUI_Update();
    assert(U16(packets.back().bytes, 7U) == 0x0110U && FigureAt(packets.back(), 0U).details_b == 29U);
    Tick(800U, false);
    assert(FigureAt(packets.back(), 2U).operate_type == operate_type_ADD);
    Tick(910U, false);
    Tick(1010U, false);
    assert(Dynamic(packets.back()) && FigureAt(packets.back(), 2U).operate_type == operate_type_Del);
    assert(FigureAt(packets.back(), 3U).details_b == 360U);
}

static void Recovery()
{
    Start(0U);
    Tick(100U);
    assert(packets.empty());
    ReceiveId(3U);
    ClientUI_Update();
    CheckPacket(packets.back(), 3U, 0x0100U, 2U);
    Tick(200U);
    Tick(600U); // exactly 500 ms since the identity frame
    size_t before = packets.size();
    Tick(700U);
    assert(packets.size() == before);
    ReceiveId(3U);
    tx_status = HAL_BUSY;
    ClientUI_Update();
    const auto failed = packets.back().bytes;
    tx_status = HAL_OK;
    ClientUI_Update();
    assert(failed == packets.back().bytes);
    CheckPacket(packets.back(), 3U, 0x0100U, 2U);
    now_us = 800000U;
    ReceiveId(105U);
    ClientUI_Update();
    CheckPacket(packets.back(), 105U, 0x0100U, 2U);
    Tick(900U);
    assert(FigureAt(packets.back(), 1U).operate_type == operate_type_ADD);
    // The sender sequence wraps naturally and does not stall scheduling.
    const uint8_t seq = packets.back().bytes[3];
    for (unsigned i = 0; i < 256U; ++i)
        assert(RefereeUITryDeleteLayer(3U, 9U));
    assert(packets.back().bytes[3] == seq);
}

int main(int argc, char** argv)
{
    assert(argc == 2);
    const std::string name = argv[1];
    if (name == "protocol")
        Protocol();
    else if (name == "identity")
        Identity();
    else if (name == "capture")
        Capture();
    else if (name == "layout")
        Layout();
    else if (name == "schedule")
        Schedule();
    else if (name == "expiry")
        Expiry();
    else if (name == "recovery")
        Recovery();
    else
        assert(false);
    std::printf("PASS %s\n", argv[1]);
}

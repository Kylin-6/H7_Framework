#ifndef CLIENT_UI_LAYOUT_H
#define CLIENT_UI_LAYOUT_H

#include "message_types.h"
#include "referee_protocol_26.h"

#include <cmath>

// 布局迁自 Infantry_Chassis HUD；只消费显示状态，不依赖机器人或板间协议。
namespace ClientUILayout
{
constexpr float kRadToDeg = 57.2957795131f;
constexpr float kRadSToRpm = 9.54929658551f;
constexpr uint8_t kStaticGroups = 6U; // 5 包七图元 + 1 包双图元
constexpr uint8_t kTextCount = 6U;

constexpr interaction_figure_t Figure(uint8_t id, uint32_t type, uint32_t color, uint32_t width,
                                      uint32_t x, uint32_t y, uint32_t c = 0U, uint32_t d = 0U, uint32_t e = 0U)
{
    interaction_figure_t graph{};
    graph.figure_name[0] = 'H';
    graph.figure_name[1] = 'U';
    graph.figure_name[2] = id;
    graph.operate_type = operate_type_ADD;
    graph.figure_type = type;
    graph.layer = 9U;
    graph.color = color;
    graph.width = width;
    graph.start_x = x;
    graph.start_y = y;
    graph.details_c = c;
    graph.details_d = d;
    graph.details_e = e;
    return graph;
}

constexpr interaction_figure_t Line(uint8_t id, uint32_t color, uint32_t width, uint32_t x, uint32_t y, uint32_t ex, uint32_t ey)
{
    return Figure(id, figure_type_Line, color, width, x, y, 0U, ex, ey);
}

constexpr interaction_figure_t Arc(uint8_t id, uint32_t color, uint32_t width, uint32_t x, uint32_t y, uint32_t radius, uint32_t start, uint32_t end)
{
    auto graph = Figure(id, figure_type_Arc, color, width, x, y, 0U, radius, radius);
    graph.details_a = start;
    graph.details_b = end;
    return graph;
}

constexpr interaction_figure_t Number(uint8_t id, uint32_t x, uint32_t y, uint32_t size, uint32_t value)
{
    auto graph = Figure(id, figure_type_Int, color_White, 1U, x, y,
                        value & 0x3FFU, (value >> 10U) & 0x7FFU, (value >> 21U) & 0x7FFU);
    graph.details_a = size;
    return graph;
}

// 1920x1080 左下角原点；刻度仅为视觉参考，尚未进行弹道/相机偏心标定。
constexpr interaction_figure_t kStaticFigures[] = {
    Figure(0U, figure_type_Circle, color_White, 2U, 960U, 540U, 15U),
    Line(1U, color_White, 1U, 660U, 540U, 1260U, 540U),
    Line(2U, color_White, 1U, 960U, 305U, 960U, 650U),
    Line(3U, color_White, 1U, 910U, 510U, 1010U, 510U),
    Line(4U, color_White, 1U, 880U, 480U, 1040U, 480U),
    Line(5U, color_White, 1U, 840U, 450U, 1080U, 450U),
    Line(6U, color_White, 1U, 760U, 420U, 1160U, 420U),
    Arc(7U, color_White, 1U, 960U, 540U, 400U, 230U, 310U),
    Arc(8U, color_White, 1U, 960U, 540U, 400U, 50U, 130U),
    Line(9U, color_White, 4U, 1235U, 771U, 1253U, 786U),
    Line(10U, color_White, 3U, 1280U, 725U, 1291U, 731U),
    Line(11U, color_White, 4U, 1297U, 663U, 1319U, 671U),
    Line(12U, color_White, 3U, 1324U, 604U, 1336U, 606U),
    Line(13U, color_White, 4U, 1319U, 540U, 1342U, 540U),
    Line(14U, color_White, 3U, 1324U, 476U, 1336U, 474U),
    Line(15U, color_White, 4U, 1297U, 417U, 1319U, 409U),
    Line(16U, color_White, 3U, 1280U, 355U, 1291U, 349U),
    Line(17U, color_White, 4U, 1235U, 309U, 1253U, 294U),
    Line(18U, color_White, 4U, 677U, 303U, 661U, 289U),
    Line(19U, color_White, 4U, 591U, 540U, 570U, 540U),
    Line(20U, color_White, 4U, 677U, 777U, 661U, 791U),
    Figure(21U, figure_type_Circle, color_White, 2U, 1560U, 720U, 77U),
    Figure(22U, figure_type_Rectangle, color_White, 1U, 1516U, 661U, 0U, 1541U, 779U),
    Figure(23U, figure_type_Rectangle, color_White, 1U, 1580U, 661U, 0U, 1605U, 779U),
    Number(24U, 1553U, 787U, 20U, 7),
    Number(25U, 1553U, 722U, 20U, 3),
    Number(26U, 1553U, 674U, 20U, 0),
    Arc(27U, color_White, 12U, 960U, 540U, 240U, 165U, 195U),
    Number(28U, 1196U, 765U, 14U, 130),
    Number(29U, 1255U, 663U, 14U, 110),
    Number(30U, 1275U, 547U, 14U, 90),
    Number(31U, 1255U, 431U, 14U, 70),
    Number(32U, 1196U, 329U, 14U, 50),
    Number(33U, 628U, 414U, 14U, 250),
    Number(34U, 697U, 294U, 14U, 500),
    Line(35U, color_Green, 2U, 630U, 320U, 505U, 175U),
    Line(36U, color_Green, 2U, 1290U, 320U, 1460U, 125U),
};

inline void BuildDynamic(const Struct_Client_UI_State& view, interaction_figure_t (&graphs)[7])
{
    // 示例未接入超级电容：保留仪表位置，删除能量填充，静态文字明确标注 N/A。
    graphs[0] = Arc(100U, color_White, 20U, 960U, 540U, 370U, 270U, 310U);
    graphs[0].operate_type = operate_type_Del;

    const bool pitch_valid = view.pitch_valid && std::isfinite(view.pitch_rad);
    const float pitch = pitch_valid ? std::fmax(-40.0f, std::fmin(40.0f, view.pitch_rad * kRadToDeg)) : 0.0f;
    const float angle = (90.0f - pitch) / kRadToDeg;
    graphs[1] = Line(101U, color_Purplish_red, 10U,
                     uint32_t(std::lround(960.0f + 348.0f * std::sin(angle))),
                     uint32_t(std::lround(540.0f + 348.0f * std::cos(angle))),
                     uint32_t(std::lround(960.0f + 386.0f * std::sin(angle))),
                     uint32_t(std::lround(540.0f + 386.0f * std::cos(angle))));
    if (!pitch_valid)
        graphs[1].operate_type = operate_type_Del;

    // A=关闭，B=等待目标/许可，C=仲裁采用视觉目标；无反馈时不亮起。
    const uint32_t autoaim = static_cast<uint32_t>(view.autoaim);
    const uint32_t start = autoaim == 1U ? 185U : autoaim == 2U ? 175U
                                                                : 165U;
    graphs[2] = Arc(102U, autoaim == 1U ? color_Pink : autoaim == 2U ? color_Cyan
                                                                     : color_Green,
                    12U, 960U, 540U, 240U, start, start + 10U);
    if (autoaim == 0U)
        graphs[2].operate_type = operate_type_Del;

    // 相对 Yaw 取负适配客户端顺时针角度，显示底盘相对云台的朝向；颜色反映实际许可/ready。
    const uint32_t ring_color = view.chassis == Enum_Client_UI_Chassis::Unknown || view.chassis == Enum_Client_UI_Chassis::Waiting ? color_Cyan : view.chassis == Enum_Client_UI_Chassis::Spin ? color_Green
                                                                                                                                                                                               : color_Pink;
    const bool yaw_valid = view.yaw_valid && std::isfinite(view.relative_yaw_rad);
    const float yaw = yaw_valid ? -std::remainder(view.relative_yaw_rad, 6.28318530718f) * kRadToDeg : 0.0f;
    const uint32_t gap = static_cast<uint32_t>(std::lround(std::fmod(360.0f + std::fmod(yaw, 360.0f), 360.0f))) % 360U;
    graphs[3] = Arc(103U, ring_color, 22U, 1560U, 720U, 90U, (gap + 30U) % 360U, (gap + 330U) % 360U);
    if (!yaw_valid)
        graphs[3] = Arc(103U, ring_color, 22U, 1560U, 720U, 90U, 0U, 360U);

    const float speed[2] = {view.left_speed_rad_s, view.right_speed_rad_s};
    for (uint8_t i = 0U; i < 2U; ++i)
    {
        // 全量程 0..7000 RPM；低速仍按真实比例显示，不伪装成 5000 RPM。
        const bool speed_valid = view.speed_valid && std::isfinite(speed[i]) && speed[i] >= 0.0f;
        const float ratio = speed_valid ? std::fmin(1.0f, (speed[i] / 7000.0f) * kRadSToRpm) : 0.0f;
        const uint32_t y = 663U + uint32_t(std::lround(114.0f * ratio));
        graphs[4U + i] = Line(104U + i, view.friction == Enum_Client_UI_Motion::Moving ? color_Green : color_Yellow,
                              6U, 1519U + 64U * i, y, 1538U + 64U * i, y);
        if (!speed_valid)
            graphs[4U + i].operate_type = operate_type_Del;
    }
    graphs[6] = Figure(106U, figure_type_Circle,
                       view.loader == Enum_Client_UI_Motion::Unknown ? color_White : view.loader == Enum_Client_UI_Motion::Moving ? color_Green
                                                                                                                                  : color_Pink,
                       4U, 1518U, 606U, 6U);
}
} // namespace ClientUILayout
#endif

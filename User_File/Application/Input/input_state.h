#ifndef RM_INPUT_STATE_H
#define RM_INPUT_STATE_H

#include "message_types.h"

#include <cstdint>

/** 当前底盘调试限幅；Remote 映射与来源仲裁使用同一组 SI 边界。 */
constexpr float INPUT_MAX_TRANSLATION_M_S = 0.5f;
constexpr float INPUT_MAX_ROTATION_RAD_S = 1.0f;

/** 固定来源选择；只有遥控模式切换逻辑应调用 InputState_Select。 */
enum class InputSource : uint8_t
{
    Remote,
    Vtm,
    Keyboard,
};

/** 解析器在任务上下文提交已经换算为 SI 的完整命令。 */
struct ControlInput
{
    ChassisCmd chassis{};
    GimbalCmd gimbal{};
    ShootCmd shoot{};
    uint32_t received_ms = 0U;
    bool valid = false;
};

/** Vision 仅提供 INS 坐标系中的绝对瞄准角；不拥有底盘和发射命令。 */
struct VisionAimInput
{
    float yaw_angle_rad = 0.0f;
    float pitch_angle_rad = 0.0f;
    uint32_t received_ms = 0U;
    bool valid = false;
};

struct InputState
{
    ControlInput remote{};
    ControlInput vtm{};
    ControlInput keyboard{};
    VisionAimInput vision{};
    InputSource selected = InputSource::Remote;
    uint32_t selected_at_ms = 0U;
    uint32_t now_ms = 0U;
    bool vision_enabled = false;
};

/** 以下接口仅供同一 ControlTask 上下文调用，不从 UART/CAN ISR 写入。 */
void InputState_Reset();
void InputState_SetTime(uint32_t now_ms);
void InputState_SubmitRemote(const ControlInput &input);
void InputState_SubmitVtm(const ControlInput &input);
void InputState_SubmitKeyboard(const ControlInput &input);
void InputState_SubmitVision(const VisionAimInput &input);
void InputState_Select(InputSource source, bool vision_enabled, uint32_t now_ms);
InputState InputState_Read();

#endif

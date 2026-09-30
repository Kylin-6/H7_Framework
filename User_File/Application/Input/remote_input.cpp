#include "remote_input.h"

#include "input_state.h"
#include "sbus.h"
#include "usart.h"

namespace
{
/* 步兵测试分支的零基通道索引；CH5 跟随须有底盘朝向反馈后再接入。 */
constexpr unsigned TRANSLATE_X = 1U; // CH2
constexpr unsigned TRANSLATE_Y = 0U; // CH1
constexpr unsigned SPEED_GEAR = 6U;  // CH7
constexpr unsigned ROTATION = 9U;    // CH10，负半轴为手动旋转
constexpr float CHANNEL_RANGE = 784.0f;
constexpr int16_t NEUTRAL_THRESHOLD = 50;
constexpr uint32_t FRAME_FRESH_MS = 50U;
constexpr uint32_t RECOVERY_MS = 200U;

bool receiver_ready;
bool armed;
uint32_t last_unhealthy_ms;

float Clamp(float value, float minimum, float maximum)
{
    return value < minimum ? minimum : (value > maximum ? maximum : value);
}

float Axis(int16_t channel)
{
    const float normalized = Clamp(static_cast<float>(channel) / CHANNEL_RANGE,
                                   -1.0f, 1.0f);
    return normalized > -0.04f && normalized < 0.04f ? 0.0f : normalized;
}

bool Neutral(const Struct_SBUS_Frame &frame)
{
    /* 解锁必须先松开平移和云台两轴；CH10 在旧遥控上是偏置开关。 */
    constexpr unsigned axes[] = {0U, 1U, 2U, 3U};
    for (unsigned index : axes)
    {
        if (frame.channels[index] < -NEUTRAL_THRESHOLD ||
            frame.channels[index] > NEUTRAL_THRESHOLD)
        {
            return false;
        }
    }
    return true;
}
}

bool RemoteInput_Init(void)
{
    armed = false;
    last_unhealthy_ms = HAL_GetTick();
    InputState_Reset();
    InputState_SetTime(last_unhealthy_ms);
    receiver_ready = SBUS_Init(&huart5);
    return receiver_ready;
}

void RemoteInput_Update(void)
{
    Struct_SBUS_Frame frame{};
    const uint32_t now = HAL_GetTick();
    InputState_SetTime(now);
    const bool healthy = receiver_ready && SBUS_ReadLatest(&frame) &&
                         now - frame.timestamp_ms <= FRAME_FRESH_MS &&
                         !frame.frame_lost && !frame.failsafe;
    if (!healthy)
    {
        last_unhealthy_ms = now;
        armed = false;
        InputState_SubmitRemote({});
        return;
    }

    if (!armed)
    {
        if (!Neutral(frame))
        {
            last_unhealthy_ms = now;
            InputState_SubmitRemote({});
            return;
        }
        if (now - last_unhealthy_ms < RECOVERY_MS)
        {
            InputState_SubmitRemote({});
            return;
        }
        armed = true;
    }

    const float gear = Clamp((static_cast<float>(frame.channels[SPEED_GEAR]) +
                              CHANNEL_RANGE) / (2.0f * CHANNEL_RANGE), 0.0f, 1.0f);
    ChassisCmd chassis{};
    chassis.velocity_x_m_s = Axis(frame.channels[TRANSLATE_X]) * gear * INPUT_MAX_TRANSLATION_M_S;
    chassis.velocity_y_m_s = -Axis(frame.channels[TRANSLATE_Y]) * gear * INPUT_MAX_TRANSLATION_M_S;
    if (frame.channels[ROTATION] < 0)
    {
        chassis.angular_velocity_rad_s = Axis(frame.channels[ROTATION]) * gear *
                                         INPUT_MAX_ROTATION_RAD_S;
    }
    if (chassis.velocity_x_m_s != 0.0f || chassis.velocity_y_m_s != 0.0f ||
        chassis.angular_velocity_rad_s != 0.0f)
    {
        chassis.mode = ChassisMode::NO_FOLLOW;
    }
    ControlInput remote{};
    remote.chassis = chassis;
    remote.received_ms = frame.timestamp_ms;
    remote.valid = true;
    InputState_SubmitRemote(remote);
}

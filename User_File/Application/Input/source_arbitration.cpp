#include "source_arbitration.h"

#include <cmath>

namespace
{
constexpr uint32_t REMOTE_MAX_AGE_MS = 50U;
constexpr uint32_t OTHER_MAX_AGE_MS = 100U;

bool Fresh(bool valid, uint32_t timestamp, uint32_t now, uint32_t max_age)
{
    return valid && now - timestamp <= max_age;
}

bool AtOrAfter(uint32_t timestamp, uint32_t boundary)
{
    return timestamp - boundary < 0x80000000U;
}

bool CommandValid(const ControlInput &input)
{
    const ChassisCmd &c = input.chassis;
    const GimbalCmd &g = input.gimbal;
    const ShootCmd &s = input.shoot;
    return (c.mode == ChassisMode::ZERO_FORCE || c.mode == ChassisMode::NO_FOLLOW ||
            c.mode == ChassisMode::FOLLOW_GIMBAL_YAW || c.mode == ChassisMode::ROTATE) &&
           std::isfinite(c.velocity_x_m_s) && std::fabs(c.velocity_x_m_s) <= INPUT_MAX_TRANSLATION_M_S &&
           std::isfinite(c.velocity_y_m_s) && std::fabs(c.velocity_y_m_s) <= INPUT_MAX_TRANSLATION_M_S &&
           std::isfinite(c.angular_velocity_rad_s) &&
           std::fabs(c.angular_velocity_rad_s) <= INPUT_MAX_ROTATION_RAD_S &&
           (g.mode == GimbalMode::DISABLED || g.mode == GimbalMode::IMU ||
            g.mode == GimbalMode::LOCK) &&
           std::isfinite(g.yaw_angle_rad) && std::isfinite(g.pitch_angle_rad) &&
           std::isfinite(g.yaw_speed_rad_s) && std::isfinite(g.pitch_speed_rad_s) &&
           (s.shoot_mode == ShootMode::OFF || s.shoot_mode == ShootMode::ON) &&
           (s.friction_mode == FrictionMode::OFF || s.friction_mode == FrictionMode::ON) &&
           (s.loader_mode == LoaderMode::STOP || s.loader_mode == LoaderMode::REVERSE ||
            s.loader_mode == LoaderMode::BURST) &&
           std::isfinite(s.friction_speed_rad_s) &&
           std::isfinite(s.loader_speed_rad_s) && std::isfinite(s.shoot_rate_hz);
}
}

InputDecision SourceArbitration_Resolve(const InputState &state)
{
    InputDecision decision{};
    decision.source = state.selected;
    /* Remote 失联始终停机，VTM/键鼠不能绕过 Remote 安全许可。 */
    if (!Fresh(state.remote.valid, state.remote.received_ms,
               state.now_ms, REMOTE_MAX_AGE_MS) || !CommandValid(state.remote))
    {
        return decision;
    }

    const ControlInput *selected = &state.remote;
    if (state.selected == InputSource::Vtm)
    {
        selected = &state.vtm;
    }
    else if (state.selected == InputSource::Keyboard)
    {
        selected = &state.keyboard;
    }
    else if (state.selected != InputSource::Remote)
    {
        return decision;
    }

    if (!Fresh(selected->valid, selected->received_ms, state.now_ms,
               selected == &state.remote ? REMOTE_MAX_AGE_MS : OTHER_MAX_AGE_MS) ||
        !AtOrAfter(selected->received_ms, state.selected_at_ms) || !CommandValid(*selected))
    {
        return decision;
    }

    decision.chassis = selected->chassis;
    decision.gimbal = selected->gimbal;
    decision.shoot = selected->shoot;
    decision.armed = true;
    if (state.vision_enabled && decision.gimbal.mode != GimbalMode::DISABLED)
    {
        const VisionAimInput &vision = state.vision;
        if (Fresh(vision.valid, vision.received_ms, state.now_ms, OTHER_MAX_AGE_MS) &&
            AtOrAfter(vision.received_ms, state.selected_at_ms) &&
            std::isfinite(vision.yaw_angle_rad) && std::isfinite(vision.pitch_angle_rad))
        {
            decision.gimbal.mode = GimbalMode::IMU;
            decision.gimbal.yaw_angle_rad = vision.yaw_angle_rad;
            decision.gimbal.pitch_angle_rad = vision.pitch_angle_rad;
            decision.gimbal.yaw_speed_rad_s = 0.0f;
            decision.gimbal.pitch_speed_rad_s = 0.0f;
        }
        else
        {
            decision.gimbal = {};
            decision.gimbal.mode = GimbalMode::LOCK;
        }
    }
    return decision;
}

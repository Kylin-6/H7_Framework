#include "ClientUI.h"
#include "message_center.h"

#include <cmath>
#include <cstring>

namespace
{
template <typename T>
bool Fresh(const TopicSnapshot<T>& sample, uint64_t now_us)
{
    return sample.valid && now_us >= sample.timestamp_us &&
           now_us - sample.timestamp_us <= ClientUIConfig::kFeedbackMaxAgeUs;
}
} // namespace

void ClientUI_Capture(void)
{
    static uint64_t last_capture_us;
    static bool captured;
    const uint64_t now_us = SYS_Timestamp_Get_Microsecond();
    if (captured && now_us - last_capture_us < 10000U)
        return;
    captured = true;
    last_capture_us = now_us;
    Struct_Client_UI_State next{};
    const auto gimbal = MessageCenter::Gimbal_Feedback_Topic.ReadWithMeta();
    if (Fresh(gimbal, now_us) && gimbal.data.ins_valid && std::isfinite(gimbal.data.pitch_rad))
    {
        next.pitch_rad = gimbal.data.pitch_rad;
        next.pitch_valid = true;
        next.pitch_timestamp_us = gimbal.timestamp_us;
    }
    const auto shoot = MessageCenter::Shoot_Feedback_Topic.ReadWithMeta();
    if (Fresh(shoot, now_us) && shoot.data.online)
    {
        next.shoot_timestamp_us = shoot.timestamp_us;
        if (std::isfinite(shoot.data.friction_left_speed_rad_s) && std::isfinite(shoot.data.friction_right_speed_rad_s))
        {
            next.left_speed_rad_s = std::fabs(shoot.data.friction_left_speed_rad_s);
            next.right_speed_rad_s = std::fabs(shoot.data.friction_right_speed_rad_s);
            next.speed_valid = true;
            next.friction = next.left_speed_rad_s > 1.0f || next.right_speed_rad_s > 1.0f
                                ? Enum_Client_UI_Motion::Moving
                                : Enum_Client_UI_Motion::Stopped;
        }
        if (std::isfinite(shoot.data.loader_speed_rad_s))
            next.loader = std::fabs(shoot.data.loader_speed_rad_s) > 1.0f
                              ? Enum_Client_UI_Motion::Moving
                              : Enum_Client_UI_Motion::Stopped;
    }
    const auto chassis = MessageCenter::Chassis_Feedback_Topic.ReadWithMeta();
    const auto command = MessageCenter::Chassis_Command_Topic.ReadWithMeta();
    if (Fresh(chassis, now_us) && Fresh(command, now_us))
    {
        next.chassis_timestamp_us = chassis.timestamp_us < command.timestamp_us ? chassis.timestamp_us : command.timestamp_us;
        const char* mode = "UNKNOWN";
        switch (command.data.mode)
        {
        case ChassisMode::ZERO_FORCE:
            mode = "STOP";
            break;
        case ChassisMode::NO_FOLLOW:
            mode = "FREE";
            break;
        case ChassisMode::FOLLOW_GIMBAL_YAW:
            mode = "FOLLOW";
            break;
        case ChassisMode::ROTATE:
            mode = "SPIN";
            break;
        }
        std::strcpy(next.mode_text, mode);
        if (chassis.data.online)
        {
            next.chassis = command.data.mode == ChassisMode::ZERO_FORCE ? Enum_Client_UI_Chassis::Stopped
                           : !chassis.data.enabled                      ? Enum_Client_UI_Chassis::Waiting
                           : command.data.mode == ChassisMode::ROTATE   ? Enum_Client_UI_Chassis::Spin
                                                                        : Enum_Client_UI_Chassis::Active;
        }
    }
    // 没有相对 Yaw、自瞄采用状态和具体机器人诊断，保留未知/空文字。
    MessageCenter::Client_UI_State_Topic.PublishAt(next, now_us);
}

#include "dmmotor.h"

#include <cstring>

static constexpr uint32_t DM_SPEED_MODE_ID_OFFSET = 0x200U;
static constexpr uint32_t DM_POSITION_SPEED_MODE_ID_OFFSET = 0x100U;
static constexpr uint32_t DM_FORCE_POSITION_MODE_ID_OFFSET = 0x300U;
static constexpr uint32_t DM_PARAMETER_ID = 0x7FFU;
static constexpr uint8_t DM_CMD_ENABLE = 0xFCU;
static constexpr uint8_t DM_CMD_DISABLE = 0xFDU;
static constexpr uint8_t DM_CMD_ZERO_POSITION = 0xFEU;
static constexpr uint8_t DM_CMD_CLEAR_ERROR = 0xFBU;
static constexpr float DM_KP_MIN = 0.0f;
static constexpr float DM_KP_MAX = 500.0f;
static constexpr float DM_KD_MIN = 0.0f;
static constexpr float DM_KD_MAX = 5.0f;

float Class_DMMotor::Clamp(float value, float min, float max)
{
    if (value < min)
    {
        return min;
    }
    if (value > max)
    {
        return max;
    }
    return value;
}

uint16_t Class_DMMotor::FloatToUint(float value, float min, float max, uint8_t bits)
{
    value = Clamp(value, min, max);
    return static_cast<uint16_t>((value - min) *
                                 static_cast<float>((1UL << bits) - 1UL) /
                                 (max - min));
}

float Class_DMMotor::UintToFloat(uint16_t value, float min, float max, uint8_t bits)
{
    return static_cast<float>(value) * (max - min) /
               static_cast<float>((1UL << bits) - 1UL) +
           min;
}

void Class_DMMotor::FeedbackCallback(FDCAN_HandleTypeDef *callback_hfdcan,
                                     uint32_t id,
                                     uint8_t *data,
                                     uint32_t len,
                                     void *context)
{
    auto *motor = static_cast<Class_DMMotor *>(context);
    if (motor == nullptr || data == nullptr || len < 8U ||
        motor->hfdcan != callback_hfdcan || motor->master_id != id ||
        (data[0] & 0x0FU) != motor->can_id)
    {
        return;
    }

    motor->state = (data[0] >> 4) & 0x0FU;
    const float decoded_position =
        UintToFloat(static_cast<uint16_t>((data[1] << 8) | data[2]),
                    -motor->position_max, motor->position_max, 16U);
    const float direction = motor->reverse ? -1.0f : 1.0f;

    if (!motor->feedback_initialized)
    {
        motor->last_position = decoded_position;
        motor->feedback_initialized = true;
    }
    else if (decoded_position - motor->last_position > motor->position_max)
    {
        --motor->total_round;
    }
    else if (decoded_position - motor->last_position < -motor->position_max)
    {
        ++motor->total_round;
    }

    motor->last_position = decoded_position;
    motor->position = direction * decoded_position;
    motor->total_position = direction *
                            (decoded_position +
                             static_cast<float>(motor->total_round) * 2.0f * motor->position_max);
    motor->velocity = direction *
                      UintToFloat(static_cast<uint16_t>((data[3] << 4) | (data[4] >> 4)),
                                  -motor->velocity_max, motor->velocity_max, 12U);
    motor->torque = direction *
                    UintToFloat(static_cast<uint16_t>(((data[4] & 0x0FU) << 8) | data[5]),
                                -motor->torque_max, motor->torque_max, 12U);
    motor->mos_temperature = static_cast<float>(data[6]);
    motor->rotor_temperature = static_cast<float>(data[7]);
}

void Class_DMMotor::SendModeCommand(uint8_t command)
{
    Struct_CAN_Tx_Msg message{};
    message.hfdcan = hfdcan;
    message.id = ControlId();
    message.len = 8U;
    std::memset(message.data, 0xFF, 7U);
    message.data[7] = command;
    CAN_Tx_Submit(&message);
}

bool Class_DMMotor::Init(FDCAN_HandleTypeDef *motor_hfdcan,
                         uint8_t motor_can_id,
                         uint16_t motor_master_id,
                         Enum_DMMotor_Mode motor_mode,
                         bool motor_reverse,
                         float motor_position_max,
                         float motor_velocity_max,
                         float motor_torque_max)
{
    if (motor_hfdcan == nullptr || motor_can_id > 0x0FU || motor_master_id > 0x7FFU)
    {
        return false;
    }

    hfdcan = motor_hfdcan;
    can_id = motor_can_id;
    master_id = motor_master_id;
    mode = motor_mode;
    reverse = motor_reverse;
    position_max = motor_position_max;
    velocity_max = motor_velocity_max;
    torque_max = motor_torque_max;
    if (position_max <= 0.0f || velocity_max <= 0.0f || torque_max <= 0.0f)
    {
        return false;
    }
    return BSP_CAN_RegisterCallback(master_id, hfdcan, FeedbackCallback, this);
}

uint32_t Class_DMMotor::ControlId() const
{
    switch (mode)
    {
    case Enum_DMMotor_Mode::POSITION_SPEED:
        return DM_POSITION_SPEED_MODE_ID_OFFSET + can_id;
    case Enum_DMMotor_Mode::SPEED:
        return DM_SPEED_MODE_ID_OFFSET + can_id;
    case Enum_DMMotor_Mode::FORCE_POSITION:
        return DM_FORCE_POSITION_MODE_ID_OFFSET + can_id;
    case Enum_DMMotor_Mode::MIT:
    default:
        return can_id;
    }
}

void Class_DMMotor::Enable()
{
    SendModeCommand(DM_CMD_ENABLE);
}

void Class_DMMotor::Disable()
{
    SendModeCommand(DM_CMD_DISABLE);
}

void Class_DMMotor::ClearError()
{
    SendModeCommand(DM_CMD_CLEAR_ERROR);
}

void Class_DMMotor::SetZeroPosition()
{
    feedback_initialized = false;
    last_position = 0.0f;
    total_round = 0;
    total_position = 0.0f;
    SendModeCommand(DM_CMD_ZERO_POSITION);
}

void Class_DMMotor::SetMode(Enum_DMMotor_Mode new_mode)
{
    const uint32_t mode_value = static_cast<uint32_t>(new_mode);
    Struct_CAN_Tx_Msg message{};
    message.hfdcan = hfdcan;
    message.id = DM_PARAMETER_ID;
    message.len = 8U;
    message.data[0] = can_id;
    message.data[1] = 0U;
    message.data[2] = 0x55U;
    message.data[3] = 0x0AU;
    std::memcpy(&message.data[4], &mode_value, sizeof(mode_value));
    CAN_Tx_Submit(&message);
    mode = new_mode;
}

void Class_DMMotor::Publish(const Struct_CAN_Tx_Msg &message)
{
    CAN_Tx_Perform(&message);
}

void Class_DMMotor::SetMIT(float position_rad,
                           float velocity_rad_s,
                           float kp,
                           float kd,
                           float torque_nm)
{
    const float direction = reverse ? -1.0f : 1.0f;
    const uint16_t position = FloatToUint(direction * position_rad,
                                          -position_max, position_max, 16U);
    const uint16_t velocity = FloatToUint(direction * velocity_rad_s,
                                          -velocity_max, velocity_max, 12U);
    const uint16_t proportional = FloatToUint(kp, DM_KP_MIN, DM_KP_MAX, 12U);
    const uint16_t derivative = FloatToUint(kd, DM_KD_MIN, DM_KD_MAX, 12U);
    const uint16_t torque = FloatToUint(direction * torque_nm,
                                        -torque_max, torque_max, 12U);

    Struct_CAN_Tx_Msg message{};
    message.hfdcan = hfdcan;
    message.id = can_id;
    message.len = 8U;
    message.data[0] = static_cast<uint8_t>(position >> 8);
    message.data[1] = static_cast<uint8_t>(position);
    message.data[2] = static_cast<uint8_t>(velocity >> 4);
    message.data[3] = static_cast<uint8_t>(((velocity & 0x0FU) << 4) |
                                           (proportional >> 8));
    message.data[4] = static_cast<uint8_t>(proportional);
    message.data[5] = static_cast<uint8_t>(derivative >> 4);
    message.data[6] = static_cast<uint8_t>(((derivative & 0x0FU) << 4) |
                                           (torque >> 8));
    message.data[7] = static_cast<uint8_t>(torque);
    Publish(message);
}

void Class_DMMotor::SetPositionSpeed(float position_rad, float velocity_rad_s)
{
    const float direction = reverse ? -1.0f : 1.0f;
    position_rad *= direction;
    velocity_rad_s *= direction;

    Struct_CAN_Tx_Msg message{};
    message.hfdcan = hfdcan;
    message.id = DM_POSITION_SPEED_MODE_ID_OFFSET + can_id;
    message.len = 8U;
    std::memcpy(&message.data[0], &position_rad, sizeof(position_rad));
    std::memcpy(&message.data[4], &velocity_rad_s, sizeof(velocity_rad_s));
    Publish(message);
}

void Class_DMMotor::SetSpeed(float speed_rad_s)
{
    if (reverse)
    {
        speed_rad_s = -speed_rad_s;
    }

    Struct_CAN_Tx_Msg message{};
    message.hfdcan = hfdcan;
    message.id = DM_SPEED_MODE_ID_OFFSET + can_id;
    message.len = sizeof(speed_rad_s);
    std::memcpy(message.data, &speed_rad_s, sizeof(speed_rad_s));
    Publish(message);
}

void Class_DMMotor::SetForcePosition(float position_rad,
                                     float velocity_limit_rad_s,
                                     float current_limit_ratio)
{
    const float direction = reverse ? -1.0f : 1.0f;
    position_rad *= direction;
    const uint16_t velocity_limit = static_cast<uint16_t>(
        Clamp(velocity_limit_rad_s, 0.0f, 100.0f) * 100.0f);
    const uint16_t current_limit = static_cast<uint16_t>(
        Clamp(current_limit_ratio, 0.0f, 1.0f) * 10000.0f);

    Struct_CAN_Tx_Msg message{};
    message.hfdcan = hfdcan;
    message.id = DM_FORCE_POSITION_MODE_ID_OFFSET + can_id;
    message.len = 8U;
    std::memcpy(&message.data[0], &position_rad, sizeof(position_rad));
    message.data[4] = static_cast<uint8_t>(velocity_limit);
    message.data[5] = static_cast<uint8_t>(velocity_limit >> 8);
    message.data[6] = static_cast<uint8_t>(current_limit);
    message.data[7] = static_cast<uint8_t>(current_limit >> 8);
    Publish(message);
}

void Class_DMMotor::SetTorque(float torque_nm)
{
    SetMIT(0.0f, 0.0f, 0.0f, 0.0f, torque_nm);
}

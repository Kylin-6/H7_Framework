#include "dji_motor.h"

namespace
{
constexpr float ENCODER_TO_DEGREE = 360.0f / 8192.0f;
constexpr float RPM_TO_DEGREE_PER_SECOND = 6.0f;
constexpr float ROTOR_SPEED_LPF_ALPHA = 0.85f;
constexpr uint8_t MAX_MOTOR_GROUPS = 15U;
constexpr uint8_t MAX_DJI_MOTORS = 24U;

struct MotorGroup
{
    Struct_CAN_Tx_Msg message{};
    Class_DJIMotor *slot_owner[4]{};
    bool initialized = false;
};

struct MotorRegistration
{
    FDCAN_HandleTypeDef *hfdcan = nullptr;
    uint32_t rx_id = 0U;
    bool used = false;
};

MotorGroup groups[MAX_MOTOR_GROUPS];
MotorRegistration registrations[MAX_DJI_MOTORS];

bool PidConfigIsUsable(const PID_InitTypeDef &config)
{
    return config.D_T > 0.0f && !Basic_Math_Is_Invalid_Float(config.D_T);
}

float DefaultGearRatio(Enum_DJIMotor_Type type)
{
    if (type == Enum_DJIMotor_Type::M2006) return 36.0f;
    if (type == Enum_DJIMotor_Type::M3508) return 19.0f;
    return 1.0f;
}

bool ResolveProtocol(Enum_DJIMotor_Type type,
                     Enum_DJIMotor_Control_Mode mode,
                     uint8_t can_id,
                     uint32_t &rx_id,
                     uint32_t &tx_id,
                     uint8_t &slot,
                     float &command_limit)
{
    if (can_id < 1U || can_id > 8U ||
        (type == Enum_DJIMotor_Type::GM6020 && can_id > 7U) ||
        (type != Enum_DJIMotor_Type::GM6020 && mode != Enum_DJIMotor_Control_Mode::CURRENT))
        return false;

    slot = (can_id - 1U) % 4U;
    if (type == Enum_DJIMotor_Type::GM6020)
    {
        rx_id = 0x204U + can_id;
        if (mode == Enum_DJIMotor_Control_Mode::VOLTAGE)
        {
            tx_id = can_id <= 4U ? 0x1FFU : 0x2FFU;
            command_limit = 25000.0f;
        }
        else
        {
            tx_id = can_id <= 4U ? 0x1FEU : 0x2FEU;
            command_limit = 16384.0f;
        }
    }
    else
    {
        rx_id = 0x200U + can_id;
        tx_id = can_id <= 4U ? 0x200U : 0x1FFU;
        command_limit = type == Enum_DJIMotor_Type::M2006 ? 10000.0f : 16384.0f;
    }
    return true;
}

int FindGroup(FDCAN_HandleTypeDef *hfdcan, uint32_t tx_id)
{
    for (uint8_t i = 0U; i < MAX_MOTOR_GROUPS; ++i)
        if (groups[i].initialized && groups[i].message.hfdcan == hfdcan &&
            groups[i].message.id == tx_id)
            return i;
    for (uint8_t i = 0U; i < MAX_MOTOR_GROUPS; ++i)
        if (!groups[i].initialized)
            return i;
    return -1;
}

int FindFreeRegistration(FDCAN_HandleTypeDef *hfdcan, uint32_t rx_id)
{
    int free_index = -1;
    for (uint8_t i = 0U; i < MAX_DJI_MOTORS; ++i)
    {
        if (registrations[i].used && registrations[i].hfdcan == hfdcan &&
            registrations[i].rx_id == rx_id)
            return -1;
        if (!registrations[i].used && free_index < 0)
            free_index = i;
    }
    return free_index;
}
}

void Class_DJIMotor::InitPid(Class_PID &pid, const PID_InitTypeDef &config)
{
    pid.Init(config.K_P, config.K_I, config.K_D, config.K_F,
             config.I_Out_Max, config.Out_Max, config.D_T, config.Dead_Zone,
             config.I_Variable_Speed_A, config.I_Variable_Speed_B,
             config.I_Separate_Threshold, config.D_First);
}

bool Class_DJIMotor::Init(const Struct_DJIMotor_Init_Config &config)
{
    if (initialized || config.hfdcan == nullptr || config.feedback_timeout_ms == 0U ||
        ((config.close_loop & DJI_MOTOR_CURRENT_LOOP) != 0U && !PidConfigIsUsable(config.current_pid)) ||
        ((config.close_loop & DJI_MOTOR_SPEED_LOOP) != 0U && !PidConfigIsUsable(config.speed_pid)) ||
        ((config.close_loop & DJI_MOTOR_ANGLE_LOOP) != 0U && !PidConfigIsUsable(config.angle_pid)) ||
        (config.close_loop & ~(DJI_MOTOR_CURRENT_LOOP | DJI_MOTOR_SPEED_LOOP | DJI_MOTOR_ANGLE_LOOP)) != 0U ||
        (config.outer_loop != DJI_MOTOR_OPEN_LOOP && (config.close_loop & config.outer_loop) == 0U) ||
        (config.angle_feedback == Enum_DJIMotor_Feedback::EXTERNAL && config.external_angle == nullptr) ||
        (config.speed_feedback == Enum_DJIMotor_Feedback::EXTERNAL && config.external_speed == nullptr))
        return false;

    uint32_t resolved_rx_id, tx_id;
    uint8_t resolved_slot;
    float resolved_limit;
    if (!ResolveProtocol(config.motor_type, config.control_mode, config.can_id,
                         resolved_rx_id, tx_id, resolved_slot, resolved_limit))
        return false;

    const int group_index = FindGroup(config.hfdcan, tx_id);
    const int registration_index = FindFreeRegistration(config.hfdcan, resolved_rx_id);
    if (group_index < 0 || registration_index < 0 ||
        (groups[group_index].initialized && groups[group_index].slot_owner[resolved_slot] != nullptr))
        return false;
    hfdcan = config.hfdcan;
    rx_id = resolved_rx_id;
    group = static_cast<uint8_t>(group_index);
    slot = resolved_slot;
    close_loop = config.close_loop;
    outer_loop = config.outer_loop;
    angle_feedback = config.angle_feedback;
    speed_feedback = config.speed_feedback;
    external_angle = config.external_angle;
    external_speed = config.external_speed;
    current_feedforward = config.current_feedforward;
    speed_feedforward = config.speed_feedforward;
    reverse = config.reverse;
    command_limit = resolved_limit;
    gear_ratio = config.gear_ratio > 0.0f && !Basic_Math_Is_Invalid_Float(config.gear_ratio)
                     ? config.gear_ratio : DefaultGearRatio(config.motor_type);
    feedback_timeout_ms = config.feedback_timeout_ms;
    has_temperature = config.motor_type != Enum_DJIMotor_Type::M2006;
    InitPid(current_pid, config.current_pid);
    InitPid(speed_pid, config.speed_pid);
    InitPid(angle_pid, config.angle_pid);

    if (!BSP_CAN_RegisterCallback(resolved_rx_id, config.hfdcan, FeedbackCallback, this))
        return false;

    MotorGroup &sender = groups[group_index];
    if (!sender.initialized)
    {
        sender.message.hfdcan = config.hfdcan;
        sender.message.id = tx_id;
        sender.message.len = 8U;
        sender.initialized = true;
    }
    sender.slot_owner[resolved_slot] = this;
    registrations[registration_index] = {config.hfdcan, resolved_rx_id, true};
    enabled = true;
    initialized = true;
    return true;
}

void Class_DJIMotor::FeedbackCallback(FDCAN_HandleTypeDef *callback_hfdcan,
                                      uint32_t callback_id,
                                      uint8_t *data,
                                      uint32_t len,
                                      void *context)
{
    Class_DJIMotor *motor = static_cast<Class_DJIMotor *>(context);
    if (motor == nullptr || data == nullptr || len != 8U ||
        callback_hfdcan != motor->hfdcan || callback_id != motor->rx_id)
        return;

    const uint16_t new_encoder = (static_cast<uint16_t>(data[0]) << 8) | data[1];
    if (!motor->feedback_initialized)
        motor->feedback_initialized = true;
    else
    {
        const int32_t delta = static_cast<int32_t>(new_encoder) - motor->last_encoder;
        if (delta > 4096) --motor->total_round;
        else if (delta < -4096) ++motor->total_round;
    }
    motor->last_encoder = new_encoder;

    const float direction = motor->reverse ? -1.0f : 1.0f;
    const int16_t rpm = static_cast<int16_t>((static_cast<uint16_t>(data[2]) << 8) | data[3]);
    motor->encoder = new_encoder;
    motor->rotor_angle = direction * new_encoder * ENCODER_TO_DEGREE;
    motor->rotor_total_angle = direction *
        (motor->total_round * 360.0f + new_encoder * ENCODER_TO_DEGREE);
    const float measured_speed = direction * rpm * RPM_TO_DEGREE_PER_SECOND;
    motor->rotor_speed = ROTOR_SPEED_LPF_ALPHA * motor->rotor_speed +
        (1.0f - ROTOR_SPEED_LPF_ALPHA) * measured_speed;
    motor->output_angle = motor->rotor_angle / motor->gear_ratio;
    motor->output_total_angle = motor->rotor_total_angle / motor->gear_ratio;
    motor->output_speed = motor->rotor_speed / motor->gear_ratio;
    motor->current_raw = static_cast<int16_t>(
        (static_cast<uint16_t>(data[4]) << 8) | data[5]);
    if (motor->has_temperature) motor->temperature = data[6];
    motor->last_feedback_tick = HAL_GetTick();
    motor->online = true;
}

void Class_DJIMotor::SetRef(float ref) { reference = ref; }

void Class_DJIMotor::ClearCommand()
{
    if (!initialized) return;
    MotorGroup &sender = groups[group];
    sender.message.data[2U * slot] = 0U;
    sender.message.data[2U * slot + 1U] = 0U;
}

bool Class_DJIMotor::ApplyWatchdog()
{
    if (!initialized) return false;
    if (online && HAL_GetTick() - last_feedback_tick > feedback_timeout_ms)
        online = false;
    if (enabled && online) return true;

    ClearCommand();
    current_pid.Set_Integral_Error(0.0f);
    speed_pid.Set_Integral_Error(0.0f);
    angle_pid.Set_Integral_Error(0.0f);
    return false;
}

void Class_DJIMotor::Control()
{
    if (!ApplyWatchdog()) return;

    float output = reference;
    if ((close_loop & DJI_MOTOR_ANGLE_LOOP) != 0U && outer_loop == DJI_MOTOR_ANGLE_LOOP)
    {
        angle_pid.Set_Target(output);
        angle_pid.Set_Now(angle_feedback == Enum_DJIMotor_Feedback::EXTERNAL
                              ? *external_angle : output_total_angle);
        angle_pid.TIM_Calculate_PeriodElapsedCallback();
        output = angle_pid.Get_Out();
    }
    if ((close_loop & DJI_MOTOR_SPEED_LOOP) != 0U &&
        (outer_loop == DJI_MOTOR_ANGLE_LOOP || outer_loop == DJI_MOTOR_SPEED_LOOP))
    {
        if (speed_feedforward != nullptr) output += *speed_feedforward;
        speed_pid.Set_Target(output);
        speed_pid.Set_Now(speed_feedback == Enum_DJIMotor_Feedback::EXTERNAL
                              ? *external_speed : output_speed);
        speed_pid.TIM_Calculate_PeriodElapsedCallback();
        output = speed_pid.Get_Out();
    }
    if (current_feedforward != nullptr) output += *current_feedforward;
    if ((close_loop & DJI_MOTOR_CURRENT_LOOP) != 0U)
    {
        const float logical_current = reverse ? -current_raw : current_raw;
        current_pid.Set_Target(output);
        current_pid.Set_Now(logical_current);
        current_pid.TIM_Calculate_PeriodElapsedCallback();
        output = current_pid.Get_Out();
    }
    if (reverse) output = -output;
    if (output > command_limit) output = command_limit;
    else if (output < -command_limit) output = -command_limit;

    const int16_t command = static_cast<int16_t>(output);
    const uint16_t raw = static_cast<uint16_t>(command);
    MotorGroup &sender = groups[group];
    sender.message.data[2U * slot] = static_cast<uint8_t>((raw >> 8) & 0xFFU);
    sender.message.data[2U * slot + 1U] = static_cast<uint8_t>(raw & 0xFFU);
}

void Class_DJIMotor::Enable() { enabled = true; }

void Class_DJIMotor::Disable()
{
    enabled = false;
    ClearCommand();
}

void Class_DJIMotor::SetOuterLoop(Enum_DJIMotor_Loop loop)
{
    if (loop == DJI_MOTOR_OPEN_LOOP || (close_loop & loop) != 0U)
        outer_loop = loop;
}

bool Class_DJIMotor::SetFeedback(Enum_DJIMotor_Loop loop,
                                 Enum_DJIMotor_Feedback source,
                                 const float *feedback)
{
    if (source == Enum_DJIMotor_Feedback::EXTERNAL && feedback == nullptr) return false;
    if (loop == DJI_MOTOR_ANGLE_LOOP)
    {
        angle_feedback = source;
        external_angle = feedback;
        return true;
    }
    if (loop == DJI_MOTOR_SPEED_LOOP)
    {
        speed_feedback = source;
        external_speed = feedback;
        return true;
    }
    return false;
}

bool Class_DJIMotor_Group::Init(Class_DJIMotor *motor1,
                                Class_DJIMotor *motor2,
                                Class_DJIMotor *motor3,
                                Class_DJIMotor *motor4)
{
    Class_DJIMotor *new_motors[4] = {motor1, motor2, motor3, motor4};
    if (motor1 == nullptr) return false;

    uint8_t count = 0U;
    bool found_null = false;
    for (uint8_t i = 0U; i < 4U; ++i)
    {
        if (new_motors[i] == nullptr)
        {
            found_null = true;
            continue;
        }
        if (found_null) return false;
        if (!new_motors[i]->initialized) return false;
        for (uint8_t j = 0U; j < i; ++j)
            if (new_motors[j] == new_motors[i]) return false;
        ++count;
    }

    for (uint8_t i = 0U; i < 4U; ++i) motors[i] = new_motors[i];
    motor_count = count;
    return true;
}

void Class_DJIMotor_Group::SetRef(float ref1, float ref2, float ref3, float ref4)
{
    const float refs[4] = {ref1, ref2, ref3, ref4};
    for (uint8_t i = 0U; i < motor_count; ++i) motors[i]->SetRef(refs[i]);
}

void Class_DJIMotor_Group::Update(float ref1, float ref2, float ref3, float ref4)
{
    SetRef(ref1, ref2, ref3, ref4);
    Control();
}

void Class_DJIMotor_Group::Control()
{
    for (uint8_t i = 0U; i < motor_count; ++i) motors[i]->Control();
}

void Class_DJIMotor_Group::Enable()
{
    for (uint8_t i = 0U; i < motor_count; ++i) motors[i]->Enable();
}

void Class_DJIMotor_Group::Disable()
{
    for (uint8_t i = 0U; i < motor_count; ++i) motors[i]->Disable();
}

bool DJIMotor_SendAll()
{
    bool success = true;
    for (uint8_t i = 0U; i < MAX_MOTOR_GROUPS; ++i)
    {
        if (!groups[i].initialized) continue;
        for (uint8_t slot = 0U; slot < 4U; ++slot)
            if (groups[i].slot_owner[slot] != nullptr)
                groups[i].slot_owner[slot]->ApplyWatchdog();
        if (!CAN_Tx_Perform(&groups[i].message)) success = false;
    }
    return success;
}

#include "lk_motor.h"

#include "sys_timestamp.h"

#include <cmath>

Class_LKMotor* Class_LKMotor::head = nullptr;

namespace
{
constexpr float RAD_PER_DEG = 0.017453292519943295f;
constexpr float DEG_PER_RAD = 57.29577951308232f;
constexpr float TWO_PI = 6.283185307179586f;
constexpr uint64_t SERVICE_QUERY_US = 20000;

uint32_t EnterCritical()
{
    const uint32_t state = __get_PRIMASK();
    __disable_irq();
    __DMB();
    return state;
}

void ExitCritical(uint32_t state)
{
    __DMB();
    __set_PRIMASK(state);
}

uint16_t ReadU16(const uint8_t* data)
{
    return static_cast<uint16_t>(data[0] | (static_cast<uint16_t>(data[1]) << 8));
}

int16_t ReadI16(const uint8_t* data)
{
    const uint16_t value = ReadU16(data);
    return static_cast<int16_t>(value < 0x8000U ? value : static_cast<int32_t>(value) - 65536);
}

void WriteI16(uint8_t* data, int16_t value)
{
    const uint16_t bits = static_cast<uint16_t>(value);
    data[0] = static_cast<uint8_t>(bits);
    data[1] = static_cast<uint8_t>(bits >> 8);
}

bool ValidPID(const PID_InitTypeDef& pid)
{
    const float values[] = {pid.K_P, pid.K_I, pid.K_D, pid.K_F, pid.I_Out_Max,
                            pid.Out_Max, pid.D_T, pid.Dead_Zone, pid.I_Variable_Speed_A,
                            pid.I_Variable_Speed_B, pid.I_Separate_Threshold, pid.D_Filter_Cutoff};
    for (float value : values)
    {
        if (!std::isfinite(value))
        {
            return false;
        }
    }
    return pid.D_T > 0 && pid.Out_Max >= 0 && pid.I_Out_Max >= 0 && pid.Dead_Zone >= 0 &&
           pid.I_Separate_Threshold >= 0 && pid.D_Filter_Cutoff >= 0 &&
           ((pid.I_Variable_Speed_A == 0 && pid.I_Variable_Speed_B == 0) ||
            (pid.I_Variable_Speed_A >= 0 && pid.I_Variable_Speed_B > pid.I_Variable_Speed_A)) &&
           (pid.D_First == PID_D_First_DISABLE || pid.D_First == PID_D_First_ENABLE);
}

void InitPID(Class_PID& pid, const PID_InitTypeDef& config)
{
    // Init 本身保留历史；先重建本地算法对象，避免掉线恢复沿用积分、微分和滤波历史。
    pid = Class_PID{};
    pid.Init(config.K_P, config.K_I, config.K_D, config.K_F, config.I_Out_Max,
             config.Out_Max, config.D_T, config.Dead_Zone, config.I_Variable_Speed_A,
             config.I_Variable_Speed_B, config.I_Separate_Threshold, config.D_First,
             config.D_Filter_Cutoff);
}

float CalculatePID(Class_PID& pid, float target, float measure)
{
    if (!std::isfinite(target) || !std::isfinite(measure))
    {
        return NAN;
    }
    pid.Set_Target(target);
    pid.Set_Now(measure);
    pid.TIM_Calculate_PeriodElapsedCallback();
    return pid.Get_Out();
}
} // namespace

bool Class_LKMotor::Init(const Struct_LKMotor_Init_Config& value)
{
    if (registration_attempted || initialized ||
        (value.hfdcan != &hfdcan1 && value.hfdcan != &hfdcan2 && value.hfdcan != &hfdcan3) ||
        (value.mode != Enum_LKMotor_Mode::SINGLE && value.mode != Enum_LKMotor_Mode::BROADCAST) ||
        (value.native_speed_control && value.native_position_control) ||
        (value.native_speed_control &&
         (value.mode != Enum_LKMotor_Mode::SINGLE || value.close_loop != LK_MOTOR_OPEN_LOOP)) ||
        (value.native_position_control &&
         (value.mode != Enum_LKMotor_Mode::SINGLE || value.close_loop != LK_MOTOR_OPEN_LOOP)) ||
        value.can_id == 0 || value.can_id > (value.mode == Enum_LKMotor_Mode::SINGLE ? 32 : 4) ||
        !std::isfinite(value.gear_ratio) || value.gear_ratio <= 0 ||
        (value.encoder_counts != 16384U && value.encoder_counts != 32768U && value.encoder_counts != 65536U) ||
        !std::isfinite(value.current_limit_raw) || value.current_limit_raw < 0 || value.current_limit_raw > 2000 ||
        value.feedback_timeout_ms == 0 || value.status_timeout_ms == 0 ||
        (value.close_loop & ~(LK_MOTOR_CURRENT_LOOP | LK_MOTOR_SPEED_LOOP | LK_MOTOR_ANGLE_LOOP)) != 0 ||
        (value.outer_loop != LK_MOTOR_OPEN_LOOP && value.outer_loop != LK_MOTOR_CURRENT_LOOP &&
         value.outer_loop != LK_MOTOR_SPEED_LOOP && value.outer_loop != LK_MOTOR_ANGLE_LOOP) ||
        (value.outer_loop != LK_MOTOR_OPEN_LOOP && (value.close_loop & value.outer_loop) == 0) ||
        (value.angle_feedback != Enum_LKMotor_Feedback::MOTOR && value.angle_feedback != Enum_LKMotor_Feedback::EXTERNAL) ||
        (value.speed_feedback != Enum_LKMotor_Feedback::MOTOR && value.speed_feedback != Enum_LKMotor_Feedback::EXTERNAL) ||
        (value.angle_feedback == Enum_LKMotor_Feedback::EXTERNAL && value.external_angle == nullptr) ||
        (value.speed_feedback == Enum_LKMotor_Feedback::EXTERNAL && value.external_speed == nullptr) ||
        ((value.close_loop & LK_MOTOR_CURRENT_LOOP) && !ValidPID(value.current_pid)) ||
        ((value.close_loop & LK_MOTOR_SPEED_LOOP) && !ValidPID(value.speed_pid)) ||
        ((value.close_loop & LK_MOTOR_ANGLE_LOOP) && !ValidPID(value.angle_pid)))
    {
        return false;
    }
    for (Class_LKMotor* motor = head; motor != nullptr; motor = motor->next)
    {
        if (motor->config.hfdcan == value.hfdcan &&
            (motor->config.can_id == value.can_id || motor->config.mode != value.mode || motor->group != nullptr))
        {
            return false;
        }
    }
    config = value;
    // BSP/Daemon 没有注销接口。资源注册开始后失败的对象必须继续保持静态生命周期，且禁止重试 Init。
    registration_attempted = true;
    if (!motion_daemon.SetTimeoutMs(config.feedback_timeout_ms) ||
        !BSP_CAN_RegisterCallback(0x140U + config.can_id, config.hfdcan, RxCallback, this) ||
        !DaemonManager::Register(motion_daemon))
    {
        return false;
    }
    const uint32_t state = EnterCritical();
    next = head;
    head = this;
    initialized = true;
    ExitCritical(state);
    return true;
}

Struct_LKMotor_Snapshot Class_LKMotor::GetFeedbackSnapshot() const
{
    const uint32_t state = EnterCritical();
    const uint64_t now = SYS_Timestamp_Get_Microsecond();
    Struct_LKMotor_Snapshot result{};
    result.feedback = feedback;
    result.motion_timestamp_us = motion_timestamp_us;
    result.status_timestamp_us = status_timestamp_us;
    result.online = initialized && motion_daemon.IsOnline();
    result.status_valid = initialized && status_seen &&
                          now - status_timestamp_us < static_cast<uint64_t>(config.status_timeout_ms) * 1000U;
    result.requested_enabled = requested_enabled;
    result.actual_enabled = result.status_valid && feedback.motor_state == 0;
    result.fault = result.status_valid && feedback.error_state != 0;
    result.ready = result.online && result.status_valid && requested_enabled &&
                   result.actual_enabled && !result.fault && !close_required && !lifecycle_pending &&
                   last_lifecycle_command == 0x88 && status_sequence != command_status_sequence &&
                   (config.mode == Enum_LKMotor_Mode::SINGLE || group != nullptr);
    ExitCritical(state);
    return result;
}

bool Class_LKMotor::IsOnline() const
{
    return GetFeedbackSnapshot().online;
}

void Class_LKMotor::InvalidateOutput()
{
    command_raw = 0;
    command_speed_raw = 0;
    reset_pid = true;
    safe_output_pending = true;
    ++output_epoch;
}

void Class_LKMotor::CheckSafety()
{
    const Struct_LKMotor_Snapshot snapshot = GetFeedbackSnapshot();
    if (!snapshot.ready)
    {
        InvalidateOutput();
    }
    if ((motion_seen && !snapshot.online) || (status_seen && !snapshot.status_valid) || snapshot.fault)
    {
        close_required = true;
    }
    if (motion_seen && !snapshot.online)
    {
        angle_initialized = false;
    }
}

void Class_LKMotor::SetRef(float ref)
{
    const uint32_t state = EnterCritical();
    reference = ref;
    if (!std::isfinite(ref))
    {
        InvalidateOutput();
    }
    ExitCritical(state);
}

bool Class_LKMotor::SetSpeed(float output_rpm)
{
    if (!initialized || !config.native_speed_control)
    {
        return false;
    }
    const uint32_t state = EnterCritical();
    CheckSafety();
    const float rotor_dps = output_rpm * config.gear_ratio * 6.0f *
                            (config.reverse ? -1.0f : 1.0f);
    const bool valid = std::isfinite(output_rpm) && std::isfinite(rotor_dps) &&
                       std::fabs(rotor_dps) <= 72000.0f && GetFeedbackSnapshot().ready;
    if (valid)
    {
        command_speed_raw = static_cast<int32_t>(std::lround(rotor_dps * 100.0f));
        safe_output_pending = false;
    }
    else
    {
        InvalidateOutput();
    }
    ExitCritical(state);
    return valid;
}

bool Class_LKMotor::SetSpeedDps(float output_dps)
{
    return SetSpeed(output_dps / 6.0f);
}

bool Class_LKMotor::MoveRelativeDegrees(float output_deg, float max_output_dps)
{
    if (!initialized || !config.native_position_control)
    {
        return false;
    }
    const uint32_t state = EnterCritical();
    CheckSafety();
    const float rotor_deg = output_deg * config.gear_ratio * (config.reverse ? -1.0f : 1.0f);
    const float rotor_max_dps = max_output_dps * config.gear_ratio;
    const bool valid = GetFeedbackSnapshot().ready && std::isfinite(rotor_deg) &&
                       std::isfinite(rotor_max_dps) && rotor_max_dps >= 1.0f &&
                       rotor_max_dps <= 65535.0f && std::fabs(rotor_deg) <= 359999.0f;
    bool submitted = false;
    if (valid)
    {
        const int32_t angle_raw = static_cast<int32_t>(std::lround(rotor_deg * 100.0f));
        const uint16_t speed_raw = static_cast<uint16_t>(std::lround(rotor_max_dps));
        Struct_CAN_Tx_Msg message{};
        message.hfdcan = config.hfdcan;
        message.id = 0x140U + config.can_id;
        message.len = 8;
        message.data[0] = 0xA8;
        message.data[2] = static_cast<uint8_t>(speed_raw);
        message.data[3] = static_cast<uint8_t>(speed_raw >> 8);
        const uint32_t bits = static_cast<uint32_t>(angle_raw);
        message.data[4] = static_cast<uint8_t>(bits);
        message.data[5] = static_cast<uint8_t>(bits >> 8);
        message.data[6] = static_cast<uint8_t>(bits >> 16);
        message.data[7] = static_cast<uint8_t>(bits >> 24);
        submitted = CAN_Tx_Submit(&message);
    }
    ExitCritical(state);
    return submitted;
}

void Class_LKMotor::ResetPID()
{
    if (config.close_loop & LK_MOTOR_CURRENT_LOOP)
    {
        InitPID(current_pid, config.current_pid);
    }
    if (config.close_loop & LK_MOTOR_SPEED_LOOP)
    {
        InitPID(speed_pid, config.speed_pid);
    }
    if (config.close_loop & LK_MOTOR_ANGLE_LOOP)
    {
        InitPID(angle_pid, config.angle_pid);
    }
}

bool Class_LKMotor::Control()
{
    if (!initialized || config.native_speed_control || config.native_position_control)
    {
        return false;
    }
    uint32_t state = EnterCritical();
    CheckSafety();
    const Struct_LKMotor_Snapshot snapshot = GetFeedbackSnapshot();
    const uint32_t epoch = output_epoch;
    float target = reference;
    const bool restart = reset_pid;
    if (snapshot.ready)
    {
        reset_pid = false;
    }
    ExitCritical(state);
    if (!snapshot.ready)
    {
        return false;
    }
    // PID 只由所属控制任务使用；服务/ISR 只登记重置，避免长临界区执行完整控制环。
    if (restart)
    {
        ResetPID();
    }
    if ((config.close_loop & LK_MOTOR_ANGLE_LOOP) && config.outer_loop == LK_MOTOR_ANGLE_LOOP)
    {
        const float measure = config.angle_feedback == Enum_LKMotor_Feedback::EXTERNAL
                                  ? *config.external_angle
                                  : snapshot.feedback.output_total_angle;
        target = CalculatePID(angle_pid, target, measure);
    }
    if ((config.close_loop & LK_MOTOR_SPEED_LOOP) &&
        (config.outer_loop == LK_MOTOR_ANGLE_LOOP || config.outer_loop == LK_MOTOR_SPEED_LOOP))
    {
        if (config.speed_feedforward != nullptr)
        {
            target += *config.speed_feedforward;
        }
        const float measure = config.speed_feedback == Enum_LKMotor_Feedback::EXTERNAL
                                  ? *config.external_speed
                                  : snapshot.feedback.output_speed;
        target = CalculatePID(speed_pid, target, measure);
    }
    if (config.current_feedforward != nullptr)
    {
        target += *config.current_feedforward;
    }
    if (config.close_loop & LK_MOTOR_CURRENT_LOOP)
    {
        const float measure = (config.reverse ? -1.0f : 1.0f) * snapshot.feedback.current_raw;
        target = CalculatePID(current_pid, target, measure);
    }
    const bool finite = std::isfinite(target);
    if (finite)
    {
        target = Basic_Math_Constrain(target, -config.current_limit_raw, config.current_limit_raw);
        if (config.reverse)
        {
            target = -target;
        }
    }
    state = EnterCritical();
    const bool valid = finite && epoch == output_epoch && GetFeedbackSnapshot().ready;
    if (valid)
    {
        command_raw = static_cast<int16_t>(target);
        safe_output_pending = false;
    }
    else
    {
        InvalidateOutput();
    }
    ExitCritical(state);
    return valid;
}

bool Class_LKMotor::SubmitCommand(uint8_t command)
{
    Struct_CAN_Tx_Msg message{};
    message.hfdcan = config.hfdcan;
    message.len = 8;
    message.id = config.mode == Enum_LKMotor_Mode::SINGLE ? 0x140U + config.can_id : 0x288U;
    message.data[config.mode == Enum_LKMotor_Mode::SINGLE ? 0 : (config.can_id - 1) * 2] = command;
    return CAN_Tx_Submit(&message);
}

bool Class_LKMotor::MaintainLifecycle(bool force)
{
    const Struct_LKMotor_Snapshot snapshot = GetFeedbackSnapshot();
    const bool enable = requested_enabled && snapshot.online && snapshot.status_valid && !snapshot.fault &&
                        !close_required && (config.mode == Enum_LKMotor_Mode::SINGLE || group != nullptr);
    const uint8_t desired = enable ? 0x88 : 0x80;
    const uint64_t now = SYS_Timestamp_Get_Microsecond();
    const bool confirmed = snapshot.status_valid && snapshot.actual_enabled == enable &&
                           last_lifecycle_command == desired && status_sequence != command_status_sequence;
    if (!force && !lifecycle_pending && confirmed)
    {
        return true;
    }
    if (!force && !lifecycle_pending && last_lifecycle_command == desired &&
        now - lifecycle_timestamp_us < SERVICE_QUERY_US)
    {
        return true;
    }
    const bool submitted = SubmitCommand(desired);
    lifecycle_pending = !submitted;
    if (submitted)
    {
        last_lifecycle_command = desired;
        command_status_sequence = status_sequence;
        lifecycle_timestamp_us = now;
        status_query_pending = true;
    }
    return submitted;
}

bool Class_LKMotor::RequestEnabled(bool enabled)
{
    if (!initialized)
    {
        return false;
    }
    const uint32_t state = EnterCritical();
    bool result = true;
    if (!request_established || requested_enabled != enabled)
    {
        request_established = true;
        requested_enabled = enabled;
        InvalidateOutput();
        lifecycle_pending = true;
        if (!enabled)
        {
            close_required = true;
        }
        const bool safe = PublishOutput();
        result = MaintainLifecycle(true) && safe;
    }
    else
    {
        result = !lifecycle_pending && !safe_output_pending;
    }
    ExitCritical(state);
    return result;
}

bool Class_LKMotor::ClearError()
{
    if (!initialized)
    {
        return false;
    }
    const uint32_t state = EnterCritical();
    clear_error_pending = !SubmitCommand(0x9B);
    const bool submitted = !clear_error_pending;
    ExitCritical(state);
    return submitted;
}

bool Class_LKMotor::PublishBus(FDCAN_HandleTypeDef* bus)
{
    // 调用者持有临界区；固定上限由 Daemon 注册容量约束。
    Struct_CAN_Tx_Msg message{};
    message.hfdcan = bus;
    message.id = 0x280;
    message.len = 8;
    for (Class_LKMotor* motor = head; motor != nullptr; motor = motor->next)
    {
        if (motor->config.hfdcan != bus || motor->config.mode != Enum_LKMotor_Mode::BROADCAST)
        {
            continue;
        }
        motor->CheckSafety();
        WriteI16(&message.data[(motor->config.can_id - 1) * 2], motor->command_raw);
    }
    const bool submitted = CAN_Tx_Perform(&message);
    if (submitted)
    {
        for (Class_LKMotor* motor = head; motor != nullptr; motor = motor->next)
        {
            if (motor->config.hfdcan == bus && motor->config.mode == Enum_LKMotor_Mode::BROADCAST)
            {
                motor->safe_output_pending = false;
            }
        }
    }
    return submitted;
}

bool Class_LKMotor::PublishOutput()
{
    if (config.mode == Enum_LKMotor_Mode::BROADCAST)
    {
        return PublishBus(config.hfdcan);
    }
    CheckSafety();
    Struct_CAN_Tx_Msg message{};
    message.hfdcan = config.hfdcan;
    message.id = 0x140U + config.can_id;
    message.len = 8;
    if (config.native_speed_control)
    {
        message.data[0] = 0xA2;
        const uint32_t bits = static_cast<uint32_t>(command_speed_raw);
        message.data[4] = static_cast<uint8_t>(bits);
        message.data[5] = static_cast<uint8_t>(bits >> 8);
        message.data[6] = static_cast<uint8_t>(bits >> 16);
        message.data[7] = static_cast<uint8_t>(bits >> 24);
    }
    else
    {
        message.data[0] = 0xA1;
        WriteI16(&message.data[4], command_raw);
    }
    const bool submitted = CAN_Tx_Perform(&message);
    if (submitted)
    {
        safe_output_pending = false;
    }
    return submitted;
}

bool Class_LKMotor::Send()
{
    if (!initialized || config.mode != Enum_LKMotor_Mode::SINGLE || config.native_position_control)
    {
        return false;
    }
    const uint32_t state = EnterCritical();
    const bool submitted = PublishOutput();
    ExitCritical(state);
    return submitted;
}

void Class_LKMotor::RxCallback(FDCAN_HandleTypeDef* bus, uint32_t id, uint8_t* data,
                               uint32_t len, void* context)
{
    auto* motor = static_cast<Class_LKMotor*>(context);
    if (motor == nullptr || !motor->initialized || data == nullptr || len != 8 ||
        bus != motor->config.hfdcan || id != 0x140U + motor->config.can_id)
    {
        return;
    }
    const bool motion = data[0] == 0xA1 || data[0] == 0xA2 || data[0] == 0xA8 || data[0] == 0x9C;
    const bool status = data[0] == 0x9A || data[0] == 0x9B;
    if (!motion && !status)
    {
        return;
    } // 启停回包不能替代新鲜状态确认。
    const uint16_t encoder = ReadU16(&data[6]);
    if (motion && encoder >= motor->config.encoder_counts)
    {
        return;
    }
    if (status && data[6] != 0 && data[6] != 0x10)
    {
        return;
    }
    const uint32_t state = EnterCritical();
    const uint64_t now = SYS_Timestamp_Get_Microsecond();
    if (motion)
    {
        // 即便 StatusTask 还没观察到掉线，收到间隔过大的新帧也不能沿用旧圈数和许可。
        if (motor->motion_seen && !motor->motion_daemon.IsOnline())
        {
            motor->angle_initialized = false;
            motor->close_required = true;
            motor->InvalidateOutput();
        }
        if (!motor->angle_initialized)
        {
            motor->accumulated_counts = encoder;
            motor->angle_initialized = true;
        }
        else
        {
            int32_t delta = static_cast<int32_t>(encoder) - motor->previous_encoder;
            const int32_t counts = static_cast<int32_t>(motor->config.encoder_counts);
            if (delta > counts / 2)
            {
                delta -= counts;
            }
            else if (delta < -counts / 2)
            {
                delta += counts;
            }
            motor->accumulated_counts += delta;
        }
        motor->previous_encoder = encoder;
        auto& feedback = motor->feedback;
        const float direction = motor->config.reverse ? -1.0f : 1.0f;
        feedback.encoder = encoder;
        feedback.current_raw = ReadI16(&data[2]);
        feedback.rotor_angle = direction * TWO_PI * encoder / motor->config.encoder_counts;
        feedback.rotor_total_angle = direction * TWO_PI * motor->accumulated_counts / motor->config.encoder_counts;
        feedback.rotor_speed = direction * RAD_PER_DEG * ReadI16(&data[4]);
        feedback.output_angle = feedback.rotor_angle / motor->config.gear_ratio;
        feedback.output_total_angle = feedback.rotor_total_angle / motor->config.gear_ratio;
        feedback.output_speed = feedback.rotor_speed / motor->config.gear_ratio;
        motor->motion_seen = true;
        motor->motion_timestamp_us = now;
        motor->motion_daemon.Feed();
    }
    else
    {
        // 状态长时间中断后到达的正常帧也不能直接放行先前保存的非零目标。
        if (motor->status_seen && now - motor->status_timestamp_us >=
                                      static_cast<uint64_t>(motor->config.status_timeout_ms) * 1000U)
        {
            motor->close_required = true;
            motor->InvalidateOutput();
        }
        motor->feedback.bus_voltage = ReadI16(&data[2]) * 0.01f;
        motor->feedback.bus_current_raw = ReadI16(&data[4]);
        motor->feedback.motor_state = data[6];
        motor->feedback.error_state = data[7];
        motor->status_seen = true;
        motor->status_timestamp_us = now;
        ++motor->status_sequence;
        if (data[6] == 0x10 && motor->last_lifecycle_command == 0x80 && !motor->lifecycle_pending)
        {
            motor->close_required = false;
        }
        if (data[6] == 0x10 || data[7] != 0)
        {
            motor->InvalidateOutput();
        }
        if (data[7] != 0)
        {
            motor->close_required = true;
        }
    }
    motor->feedback.temperature = static_cast<int8_t>(data[1] < 128 ? data[1] : static_cast<int>(data[1]) - 256);
    ExitCritical(state);
}

void Class_LKMotor::ServiceAll()
{
    for (Class_LKMotor* motor = head; motor != nullptr; motor = motor->next)
    {
        const uint32_t state = EnterCritical();
        motor->CheckSafety();
        if (motor->safe_output_pending)
        {
            (void) motor->PublishOutput();
        }
        (void) motor->MaintainLifecycle(false);
        if (motor->clear_error_pending)
        {
            motor->clear_error_pending = !motor->SubmitCommand(0x9B);
        }
        const uint64_t now = SYS_Timestamp_Get_Microsecond();
        if (!motor->poll_started || now - motor->poll_timestamp_us >= SERVICE_QUERY_US)
        {
            motor->poll_started = true;
            motor->poll_timestamp_us = now;
            motor->status_query_pending = true;
            motor->motion_query_pending = motor->config.native_position_control ||
                                          !motor->GetFeedbackSnapshot().ready;
        }
        if (motor->status_query_pending)
        {
            motor->status_query_pending = !motor->SubmitCommand(0x9A);
        }
        if (!motor->config.native_position_control && motor->GetFeedbackSnapshot().ready)
        {
            motor->motion_query_pending = false;
        }
        if (motor->motion_query_pending)
        {
            motor->motion_query_pending = !motor->SubmitCommand(0x9C);
        }
        ExitCritical(state);
    }
}

bool Class_LKMotor_Group::Init(Class_LKMotor* motor1, Class_LKMotor* motor2,
                               Class_LKMotor* motor3, Class_LKMotor* motor4)
{
    if (bus != nullptr || motor1 == nullptr)
    {
        return false;
    }
    Class_LKMotor* members[] = {motor1, motor2, motor3, motor4};
    uint8_t size = 0;
    bool end = false;
    for (Class_LKMotor* motor : members)
    {
        if (motor == nullptr)
        {
            end = true;
            continue;
        }
        if (end || !motor->initialized || motor->group != nullptr ||
            motor->config.mode != Enum_LKMotor_Mode::BROADCAST || motor->config.hfdcan != motor1->config.hfdcan)
        {
            return false;
        }
        for (uint8_t i = 0; i < size; ++i)
        {
            if (members[i] == motor)
            {
                return false;
            }
        }
        ++size;
    }
    uint8_t registered = 0;
    for (Class_LKMotor* motor = Class_LKMotor::head; motor != nullptr; motor = motor->next)
    {
        if (motor->config.hfdcan == motor1->config.hfdcan)
        {
            ++registered;
        }
    }
    if (registered != size)
    {
        return false;
    }
    const uint32_t state = EnterCritical();
    bus = motor1->config.hfdcan;
    count = size;
    for (uint8_t i = 0; i < size; ++i)
    {
        motors[i] = members[i];
        members[i]->group = this;
    }
    ExitCritical(state);
    return true;
}

void Class_LKMotor_Group::SetRef(float ref1, float ref2, float ref3, float ref4)
{
    const float values[] = {ref1, ref2, ref3, ref4};
    for (uint8_t i = 0; i < count; ++i)
    {
        motors[i]->SetRef(values[i]);
    }
}

bool Class_LKMotor_Group::Control()
{
    bool ready = count != 0;
    for (uint8_t i = 0; i < count; ++i)
    {
        ready = motors[i]->Control() && ready;
    }
    return ready;
}

bool Class_LKMotor_Group::Control(float ref1, float ref2, float ref3, float ref4)
{
    SetRef(ref1, ref2, ref3, ref4);
    const bool ready = Control();
    return Send() && ready;
}

bool Class_LKMotor_Group::Send()
{
    if (bus == nullptr)
    {
        return false;
    }
    const uint32_t state = EnterCritical();
    const bool submitted = Class_LKMotor::PublishBus(bus);
    ExitCritical(state);
    return submitted;
}

bool Class_LKMotor_Group::RequestEnabled(bool enabled)
{
    bool submitted = count != 0;
    for (uint8_t i = 0; i < count; ++i)
    {
        submitted = motors[i]->RequestEnabled(enabled) && submitted;
    }
    return submitted;
}

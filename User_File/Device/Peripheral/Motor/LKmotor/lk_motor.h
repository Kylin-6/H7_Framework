/** @file lk_motor.h
 * @brief LK MF/MH/MG 系列：CAN 电流/速度/增量位置控制与设备安全服务。
 */
#ifndef LK_MOTOR_H
#define LK_MOTOR_H

#include "alg_pid.h"
#include "bsp_can.h"
#include "daemon.h"

enum class Enum_LKMotor_Mode : uint8_t
{
    SINGLE,
    BROADCAST
};
enum class Enum_LKMotor_Feedback : uint8_t
{
    MOTOR,
    EXTERNAL
};
enum Enum_LKMotor_Loop : uint8_t
{
    LK_MOTOR_OPEN_LOOP = 0,
    LK_MOTOR_CURRENT_LOOP = 1 << 0,
    LK_MOTOR_SPEED_LOOP = 1 << 1,
    LK_MOTOR_ANGLE_LOOP = 1 << 2,
};

struct Struct_LKMotor_Init_Config
{
    FDCAN_HandleTypeDef* hfdcan = nullptr;
    uint8_t can_id = 1; ///< 节点编号；单机 1~32，广播 1~4。
    Enum_LKMotor_Mode mode = Enum_LKMotor_Mode::SINGLE;
    bool native_speed_control = false; ///< 单机 A2 速度闭环；广播仍只支持电流。
    bool native_position_control = false; ///< 单机 A8 增量位置闭环；命令只提交一次。
    float gear_ratio = 0.0f;         ///< 必填：转子/输出轴减速比。
    uint32_t encoder_counts = 0U;   ///< 必填：CAN 每转子圈计数，不是硬件位数。
    float current_limit_raw = 0.0f;   ///< 应用幅值上限 [0,2000]；0 禁止非零输出。
    bool reverse = false;
    uint32_t feedback_timeout_ms = 50;
    uint32_t status_timeout_ms = 100;
    uint8_t close_loop = LK_MOTOR_OPEN_LOOP;
    Enum_LKMotor_Loop outer_loop = LK_MOTOR_OPEN_LOOP;
    PID_InitTypeDef current_pid{}; ///< 目标、测量和输出均为电流 raw。
    PID_InitTypeDef speed_pid{};   ///< 目标和测量为输出轴 rad/s。
    PID_InitTypeDef angle_pid{};   ///< 目标和测量为输出轴累计 rad。
    Enum_LKMotor_Feedback angle_feedback = Enum_LKMotor_Feedback::MOTOR;
    Enum_LKMotor_Feedback speed_feedback = Enum_LKMotor_Feedback::MOTOR;
    const float* external_angle = nullptr; ///< 指针须长期有效，调用方统一正方向和零点。
    const float* external_speed = nullptr;
    const float* speed_feedforward = nullptr;   ///< rad/s，仅执行速度环时使用。
    const float* current_feedforward = nullptr; ///< raw，电流环前（未启用时直接加到指令）。
};

struct Struct_LKMotor_Feedback
{
    uint16_t encoder = 0;
    int16_t current_raw = 0; ///< 原始报文符号，不声明为 A 或 N*m。
    int8_t temperature = 0;
    float rotor_angle = 0.0f; ///< rad，含方向配置。
    float rotor_total_angle = 0.0f;
    float rotor_speed = 0.0f; ///< rad/s。
    float output_angle = 0.0f;
    float output_total_angle = 0.0f;
    float output_speed = 0.0f;
    float bus_voltage = 0.0f;    ///< V；状态 1 回复。
    int16_t bus_current_raw = 0; ///< 独立保留，不作为转矩电流。
    uint8_t motor_state = 0x10;
    uint8_t error_state = 0;
};

struct Struct_LKMotor_Snapshot
{
    Struct_LKMotor_Feedback feedback{};
    uint64_t motion_timestamp_us = 0;
    uint64_t status_timestamp_us = 0;
    bool online = false;
    bool status_valid = false;
    bool requested_enabled = false;
    bool actual_enabled = false; ///< 仅在状态新鲜且报告开启时为 true。
    bool fault = false;
    bool ready = false;
};

class Class_LKMotor_Group;

/** 静态生命周期；初始化在任务启动前完成，一个控制任务拥有本实例。 */
class Class_LKMotor
{
  public:
    Class_LKMotor() = default;
    Class_LKMotor(const Class_LKMotor&) = delete;
    Class_LKMotor& operator=(const Class_LKMotor&) = delete;
    bool Init(const Struct_LKMotor_Init_Config& config);
    void SetRef(float ref);
    bool SetSpeed(float output_rpm); ///< A2 减速后输出轴速度目标，单位 rpm；仅单机原生速度模式。
    bool SetSpeedDps(float output_dps); ///< A2 减速后输出轴速度目标，单位 dps；仅单机原生速度模式。
    bool MoveRelativeDegrees(float output_deg, float max_output_dps); ///< A8 单次增量，均为减速后输出轴单位。
    bool Control();                    ///< 只计算；未就绪/非法输入时清零并返回 false。
    bool Send();                       ///< 仅单机模式；返回周期槽提交结果，未就绪时提交对应模式的零目标。
    bool RequestEnabled(bool enabled); ///< 返回当前必要操作的软件提交结果，不是使能确认。
    bool ClearError();                 ///< 显式清错；失败由 ServiceAll 补交，不自动清错。
    Struct_LKMotor_Snapshot GetFeedbackSnapshot() const;
    bool IsOnline() const;
    static void ServiceAll(); ///< 现有 StatusTask 100 Hz 调用。

  private:
    friend class Class_LKMotor_Group;
    static void RxCallback(FDCAN_HandleTypeDef*, uint32_t, uint8_t*, uint32_t, void*);
    void InvalidateOutput(); ///< 调用者持有临界区；PID 重置推迟到控制任务。
    void CheckSafety();
    void ResetPID();
    bool SubmitCommand(uint8_t command);
    bool MaintainLifecycle(bool force);
    bool PublishOutput(); ///< 支持单机或本总线广播安全覆盖。
    static bool PublishBus(FDCAN_HandleTypeDef* bus);

    Struct_LKMotor_Init_Config config{};
    Struct_LKMotor_Feedback feedback{};
    Class_PID current_pid{};
    Class_PID speed_pid{};
    Class_PID angle_pid{};
    Daemon motion_daemon{50};
    Class_LKMotor* next = nullptr;
    static Class_LKMotor* head;
    Class_LKMotor_Group* group = nullptr;
    bool initialized = false;
    bool registration_attempted = false;
    bool requested_enabled = false;
    bool request_established = false;
    bool close_required = true;
    bool safe_output_pending = true;
    bool lifecycle_pending = true;
    bool clear_error_pending = false;
    bool reset_pid = true;
    bool motion_seen = false;
    bool angle_initialized = false;
    bool status_seen = false;
    bool poll_started = false;
    bool status_query_pending = false;
    bool motion_query_pending = false;
    uint16_t previous_encoder = 0;
    int64_t accumulated_counts = 0;
    uint32_t output_epoch = 0;
    uint32_t status_sequence = 0;
    uint32_t command_status_sequence = 0;
    uint8_t last_lifecycle_command = 0;
    uint64_t motion_timestamp_us = 0;
    uint64_t status_timestamp_us = 0;
    uint64_t lifecycle_timestamp_us = 0;
    uint64_t poll_timestamp_us = 0;
    float reference = 0.0f;
    int16_t command_raw = 0;
    int32_t command_speed_raw = 0;
};

/** 一个广播组独占一条总线的全部已注册 LK 广播电机；参数顺序决定目标顺序。 */
class Class_LKMotor_Group
{
  public:
    Class_LKMotor_Group() = default;
    Class_LKMotor_Group(const Class_LKMotor_Group&) = delete;
    Class_LKMotor_Group& operator=(const Class_LKMotor_Group&) = delete;
    bool Init(Class_LKMotor* motor1, Class_LKMotor* motor2 = nullptr,
              Class_LKMotor* motor3 = nullptr, Class_LKMotor* motor4 = nullptr);
    void SetRef(float ref1, float ref2 = 0, float ref3 = 0, float ref4 = 0);
    bool Control(); ///< 计算所有成员，不发送。
    bool Control(float ref1, float ref2 = 0, float ref3 = 0, float ref4 = 0);
    bool Send(); ///< 即使部分成员未就绪，仍发布其余成员目标。
    bool RequestEnabled(bool enabled);

  private:
    Class_LKMotor* motors[4]{};
    FDCAN_HandleTypeDef* bus = nullptr;
    uint8_t count = 0;
};

#endif

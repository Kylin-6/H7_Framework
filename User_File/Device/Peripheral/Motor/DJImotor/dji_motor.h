#ifndef DJI_MOTOR_H
#define DJI_MOTOR_H

#include "alg_pid.h"
#include "bsp_can.h"

#include <stdint.h>

enum class Enum_DJIMotor_Type : uint8_t { M2006, M3508, GM6020 };
enum class Enum_DJIMotor_Control_Mode : uint8_t { CURRENT, VOLTAGE };
enum Enum_DJIMotor_Loop : uint8_t
{
    DJI_MOTOR_OPEN_LOOP = 0U,
    DJI_MOTOR_CURRENT_LOOP = 1U << 0,
    DJI_MOTOR_SPEED_LOOP = 1U << 1,
    DJI_MOTOR_ANGLE_LOOP = 1U << 2,
};
enum class Enum_DJIMotor_Feedback : uint8_t { MOTOR, EXTERNAL };

struct Struct_DJIMotor_Init_Config
{
    FDCAN_HandleTypeDef *hfdcan;
    uint8_t can_id;
    Enum_DJIMotor_Type motor_type;
    uint8_t close_loop;
    Enum_DJIMotor_Loop outer_loop;
    PID_InitTypeDef current_pid;
    PID_InitTypeDef speed_pid;
    PID_InitTypeDef angle_pid;
    Enum_DJIMotor_Control_Mode control_mode = Enum_DJIMotor_Control_Mode::CURRENT;
    float gear_ratio = 0.0f;
    uint32_t feedback_timeout_ms = 20U;
    bool reverse = false;
    Enum_DJIMotor_Feedback angle_feedback = Enum_DJIMotor_Feedback::MOTOR;
    Enum_DJIMotor_Feedback speed_feedback = Enum_DJIMotor_Feedback::MOTOR;
    const float *external_angle = nullptr;
    const float *external_speed = nullptr;
    const float *current_feedforward = nullptr;
    const float *speed_feedforward = nullptr;
};

class Class_DJIMotor
{
public:
    bool Init(const Struct_DJIMotor_Init_Config &config);
    void SetRef(float ref);
    void Control();
    void Enable();
    void Disable();
    void SetOuterLoop(Enum_DJIMotor_Loop loop);
    bool SetFeedback(Enum_DJIMotor_Loop loop, Enum_DJIMotor_Feedback source,
                     const float *feedback = nullptr);

    uint16_t encoder = 0U;
    float rotor_angle = 0.0f;
    float rotor_total_angle = 0.0f;
    float rotor_speed = 0.0f;
    float output_angle = 0.0f;
    float output_total_angle = 0.0f;
    float output_speed = 0.0f;
    int16_t current_raw = 0;
    uint8_t temperature = 0U;
    volatile uint32_t last_feedback_tick = 0U;
    volatile bool online = false;

    Class_PID current_pid;
    Class_PID speed_pid;
    Class_PID angle_pid;

private:
    friend bool DJIMotor_SendAll();
    friend class Class_DJIMotor_Group;
    static void FeedbackCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t id,
                                 uint8_t *data, uint32_t len, void *context);
    static void InitPid(Class_PID &pid, const PID_InitTypeDef &config);
    bool ApplyWatchdog();
    void ClearCommand();

    FDCAN_HandleTypeDef *hfdcan = nullptr;
    uint32_t rx_id = 0U;
    uint8_t group = 0U;
    uint8_t slot = 0U;
    uint8_t close_loop = DJI_MOTOR_OPEN_LOOP;
    Enum_DJIMotor_Loop outer_loop = DJI_MOTOR_OPEN_LOOP;
    Enum_DJIMotor_Feedback angle_feedback = Enum_DJIMotor_Feedback::MOTOR;
    Enum_DJIMotor_Feedback speed_feedback = Enum_DJIMotor_Feedback::MOTOR;
    const float *external_angle = nullptr;
    const float *external_speed = nullptr;
    const float *current_feedforward = nullptr;
    const float *speed_feedforward = nullptr;
    float reference = 0.0f;
    float command_limit = 0.0f;
    float gear_ratio = 1.0f;
    uint32_t feedback_timeout_ms = 20U;
    bool has_temperature = false;
    bool reverse = false;
    bool enabled = false;
    bool initialized = false;
    bool feedback_initialized = false;
    uint16_t last_encoder = 0U;
    int32_t total_round = 0;
};

class Class_DJIMotor_Group
{
public:
    bool Init(Class_DJIMotor *motor1,
              Class_DJIMotor *motor2 = nullptr,
              Class_DJIMotor *motor3 = nullptr,
              Class_DJIMotor *motor4 = nullptr);
    void SetRef(float ref1,
                float ref2 = 0.0f,
                float ref3 = 0.0f,
                float ref4 = 0.0f);
    void Update(float ref1,
                float ref2 = 0.0f,
                float ref3 = 0.0f,
                float ref4 = 0.0f);
    void Control();
    void Enable();
    void Disable();

private:
    Class_DJIMotor *motors[4]{};
    uint8_t motor_count = 0U;
};

bool DJIMotor_SendAll();

#endif

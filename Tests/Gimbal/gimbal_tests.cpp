#include "Gimbal.h"
#include "message_center.h"
#include "sys_timestamp.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <vector>

#define CHECK(x) do { if (!(x)) { std::fprintf(stderr, "FAIL %d: %s\n", __LINE__, #x); std::exit(1); } } while (0)
FDCAN_HandleTypeDef hfdcan1{1}, hfdcan2{2}, hfdcan3{3};
uint32_t test_irq_mask;
uint64_t test_timestamp_us;
Class_Timestamp SYS_Timestamp;
extern "C" uint64_t SYS_Timestamp_Get_Microsecond() { return test_timestamp_us; }
struct Receiver { uint32_t id; FDCAN_HandleTypeDef *bus; CAN_RxCallback_t fn; void *context; };
static std::vector<Receiver> receivers;
static std::vector<Struct_CAN_Tx_Msg> discrete, periodic;
static bool submit_ok = true, perform_ok = true, registration_ok = true;
static unsigned submit_attempts, perform_attempts;
static unsigned registration_limit = 2;
static Struct_Gimbal_Config cfg;
static INS_State ins{};
static GimbalCmd command{};
static float motor_position = -0.2f, motor_velocity = 0.0f;

bool BSP_CAN_RegisterCallback(uint32_t id, FDCAN_HandleTypeDef *bus, CAN_RxCallback_t fn, void *context)
{
    if (!registration_ok || receivers.size() >= registration_limit) return false;
    receivers.push_back({id, bus, fn, context});
    return true;
}
bool CAN_Tx_Submit(const Struct_CAN_Tx_Msg *message)
{
    ++submit_attempts;
    if (submit_ok) discrete.push_back(*message);
    return submit_ok;
}
bool CAN_Tx_Perform(const Struct_CAN_Tx_Msg *message)
{
    ++perform_attempts;
    if (perform_ok) periodic.push_back(*message);
    return perform_ok;
}
static bool Near(float a, float b, float tolerance = 0.02f) { return std::fabs(a - b) < tolerance; }
static uint16_t Encode(float value, float maximum, unsigned bits)
{
    return static_cast<uint16_t>((value + maximum) * ((1U << bits) - 1U) / (2 * maximum));
}
static void Receive(const Struct_Gimbal_Motor_Config &c, int state, float position, float velocity)
{
    const uint16_t p = Encode(position, c.position_max, 16);
    const uint16_t v = Encode(velocity, c.velocity_max, 12);
    const uint16_t t = Encode(0, c.torque_max, 12);
    uint8_t frame[8] = {static_cast<uint8_t>((state << 4) | (c.id & 15)),
        static_cast<uint8_t>(p >> 8), static_cast<uint8_t>(p), static_cast<uint8_t>(v >> 4),
        static_cast<uint8_t>((v << 4) | (t >> 8)), static_cast<uint8_t>(t), 25, 26};
    for (const auto &r : receivers)
        if (r.id == c.feedback_id && r.bus == c.bus) r.fn(c.bus, c.feedback_id, frame, 8, r.context);
}
static void Tick(uint64_t elapsed = 1000, bool imu = true, int yaw = 1, int pitch = 1)
{
    test_timestamp_us += elapsed;
    if (imu) MessageCenter::INS_State_Topic.Publish(ins);
    if (yaw >= 0) Receive(cfg.yaw, yaw, 0, 0);
    if (pitch >= 0) Receive(cfg.pitch, pitch, motor_position, motor_velocity);
    Gimbal_Update();
}
static void Publish() { MessageCenter::Gimbal_Command_Topic.Publish(command); }
static void Init(bool tuned = true)
{
    cfg = Gimbal_Default_Config();
    if (tuned) { cfg.yaw_speed_kp = 1; cfg.yaw_torque_limit = 2; }
    CHECK(Gimbal_Init(cfg));
    CHECK(discrete.empty()); // No enable, zeroing or persistent writes during initialization.
    CHECK(receivers.size() == 2);
    CHECK(receivers[0].bus == &hfdcan2 && receivers[0].id == 0x101);
    CHECK(receivers[1].bus == &hfdcan1 && receivers[1].id == 0x102);
}
static void Activate()
{
    command.mode = GimbalMode::IMU;
    Publish();
    Tick(1000, true, 0, 0);
    CHECK(Gimbal.status == Gimbal_Status_ENABLING);
    CHECK(discrete.size() >= 2);
    CHECK(discrete.back().data[7] == 0xfc);
    Tick();
    for (unsigned i = 0; i < 99; ++i) Tick();
    CHECK(Gimbal.status == Gimbal_Status_ENABLING);
    Tick();
    CHECK(Gimbal.status == Gimbal_Status_READY);
}
static Struct_CAN_Tx_Msg Last(const Struct_Gimbal_Motor_Config &c)
{
    for (auto it = periodic.rbegin(); it != periodic.rend(); ++it)
        if (it->hfdcan == c.bus && it->id == c.id) return *it;
    CHECK(false);
    return {};
}
struct MIT { float p, v, kp, kd, torque; };
static MIT Decode(const Struct_Gimbal_Motor_Config &c)
{
    const auto f = Last(c);
    CHECK(f.len == 8);
    auto value = [](unsigned raw, float max, unsigned bits) {
        return raw * (2 * max) / ((1U << bits) - 1U) - max;
    };
    return {value((f.data[0] << 8) | f.data[1], c.position_max, 16),
        value((f.data[2] << 4) | (f.data[3] >> 4), c.velocity_max, 12),
        (((f.data[3] & 15) << 8) | f.data[4]) * 500.0f / 4095,
        ((f.data[5] << 4) | (f.data[6] >> 4)) * 5.0f / 4095,
        value(((f.data[6] & 15) << 8) | f.data[7], c.torque_max, 12)};
}
static void Zero()
{
    for (const auto &c : {cfg.yaw, cfg.pitch})
    {
        const auto f = Decode(c);
        CHECK(f.kp == 0 && f.kd == 0 && Near(f.torque, 0));
    }
}
static void TestConfig()
{
    auto base = Gimbal_Default_Config();
    for (int scenario = 0; scenario < 14; ++scenario)
    {
        auto c = base;
        switch (scenario)
        {
        case 0: c.yaw.bus = nullptr; break;
        case 1: c.yaw.id = 0; break;
        case 2: c.pitch.feedback_id = 0x800; break;
        case 3: c.pitch = c.yaw; break;
        case 4: c.yaw.position_max = 0; break;
        case 5: c.yaw.velocity_max = std::numeric_limits<float>::quiet_NaN(); break;
        case 6: c.pitch.torque_max = -1; break;
        case 7: c.pitch_min = c.pitch_max; break;
        case 8: c.pitch_min = -20; break;
        case 9: c.pitch_kp = 501; break;
        case 10: c.pitch_speed_limit = 100; break;
        case 11: c.yaw_gyro_axis = static_cast<GimbalGyroAxis>(3); break;
        case 12: c.yaw_torque_limit = 100; break;
        case 13: c.pitch_motor_per_imu = 0; break;
        }
        CHECK(!Gimbal_Init(c));
        CHECK(Gimbal.status == Gimbal_Status_CONFIG_ERROR);
        Gimbal_Update();
        CHECK(receivers.empty() && discrete.empty() && periodic.empty());
    }
}
static void TestControl()
{
    Init(); Activate();
    ins.yaw_rad = -179 * 3.14159265f / 180;
    command.yaw_angle_rad = 179 * 3.14159265f / 180;
    command.yaw_speed_rad_s = 0.5f;
    ins.gyro_z_rad_s = 0.1f;
    Publish(); Tick();
    CHECK(Near(Decode(cfg.yaw).torque, (-2 * 3.14159265f / 180) * 8 + 0.5f - 0.1f));
    command.yaw_speed_rad_s = 100;
    Publish(); Tick();
    CHECK(Near(Decode(cfg.yaw).torque, 2));
    command.yaw_speed_rad_s = -100;
    Publish(); Tick();
    CHECK(Near(Decode(cfg.yaw).torque, -2));
}
static void TestPitch(bool reverse)
{
    cfg = Gimbal_Default_Config();
    cfg.pitch.reverse = reverse;
    cfg.yaw.reverse = reverse;
    cfg.yaw_speed_kp = 1;
    cfg.pitch_motor_per_imu = 2;
    CHECK(Gimbal_Init(cfg)); Activate();
    ins.pitch_rad = 0.1f;
    ins.gyro_y_rad_s = 0.2f;
    ins.gyro_x_rad_s = 10; // Must not accidentally use reference project's X-axis convention.
    motor_velocity = reverse ? -0.4f : 0.4f;
    command.yaw_angle_rad = 0.1f;
    command.pitch_angle_rad = 0.2f;
    command.pitch_speed_rad_s = 0.3f;
    Publish(); Tick();
    const float direction = reverse ? -1 : 1;
    const auto feedback = Gimbal.Pitch_Motor.GetFeedbackSnapshot().feedback;
    auto output = Decode(cfg.pitch);
    CHECK(Near(Decode(cfg.yaw).torque, direction * 0.8f));
    CHECK(Near(output.p, direction * (feedback.position + 0.2f)));
    CHECK(Near(output.v, direction * (feedback.velocity + 0.2f), 0.03f));
    CHECK(Near(output.kp, 20, 0.13f) && Near(output.kd, 1, 0.002f));
    command.pitch_angle_rad = 100; command.pitch_speed_rad_s = 100;
    Publish(); Tick(); output = Decode(cfg.pitch);
    CHECK(Near(output.p, direction * cfg.pitch_max));
    CHECK(Near(output.v, direction * cfg.pitch_speed_limit, 0.03f));
    command.pitch_angle_rad = -100; command.pitch_speed_rad_s = -100;
    Publish(); Tick(); output = Decode(cfg.pitch);
    CHECK(Near(output.p, direction * cfg.pitch_min));
    CHECK(Near(output.v, -direction * cfg.pitch_speed_limit, 0.03f));
}
static void TestModes()
{
    Init(); Tick(); Zero();
    for (const auto &f : discrete) CHECK(f.data[7] == 0xfd);
    discrete.clear(); Activate();
    ins.yaw_rad = 0.2f; ins.pitch_rad = 0.1f;
    command.mode = GimbalMode::LOCK; command.yaw_angle_rad = 2;
    Publish(); Tick();
    CHECK(Near(Gimbal.Target_Yaw_Angle, 0.2f));
    CHECK(Near(Gimbal.Target_Pitch_Angle, 0.1f));
    ins.yaw_rad = 0.25f; Tick();
    CHECK(Near(Decode(cfg.yaw).torque, -0.4f));
    command.yaw_angle_rad = 3; Publish(); Tick();
    CHECK(Near(Gimbal.Target_Yaw_Angle, 0.2f));
    command.mode = GimbalMode::DISABLED; Publish(); Tick(); Zero();
    discrete.clear();
    Tick(101000, true, -1, -1); DaemonManager::CheckAll();
    for (const auto &f : discrete) CHECK(f.data[7] == 0xfd);
    CHECK(Gimbal.status == Gimbal_Status_DISABLE);
}
static void TestRecovery()
{
    Init(); Activate();
    Gimbal.Yaw_Speed_PID.Set_K_I(1);
    command.yaw_angle_rad = 1; Publish(); Tick();
    CHECK(Gimbal.Yaw_Speed_PID.Get_Integral_Error() != 0);
    Tick(11000, false);
    CHECK(Gimbal.status == Gimbal_Status_FAULT); Zero();
    ins.yaw_rad = 0.3f; ins.pitch_rad = 0.2f;
    Tick(999000, true, 0, 0);
    CHECK(Gimbal.status == Gimbal_Status_FAULT);
    Tick(1000, true, 0, 0);
    CHECK(Gimbal.status == Gimbal_Status_ENABLING);
    Tick();
    for (int i = 0; i < 100; ++i) Tick();
    CHECK(Gimbal.status == Gimbal_Status_READY);
    CHECK(Gimbal.Yaw_Speed_PID.Get_Integral_Error() == 0);
    CHECK(Near(Gimbal.Target_Yaw_Angle, 0.3f));
    Tick(); CHECK(Near(Decode(cfg.yaw).torque, 0)); // Old yaw=1 must not return.
    command.yaw_angle_rad = 0.4f; command.pitch_angle_rad = 0.2f; Publish(); Tick();
    CHECK(Near(Decode(cfg.yaw).torque, 0.8f));
    Tick(100000, true, -1, -1);
    CHECK(Gimbal.status == Gimbal_Status_FAULT); Zero();
    Tick(1000000, true, 8, 1);
    CHECK(Gimbal.status == Gimbal_Status_FAULT);
    for (const auto &f : discrete) CHECK(f.data[7] != 0xfb && f.data[7] != 0xfe);
}
static void TestStable()
{
    Init(); command.mode = GimbalMode::LOCK; Publish();
    Tick(); Tick(99000);
    CHECK(Gimbal.status == Gimbal_Status_ENABLING);
    Tick(1000, true, 0, 1); // A disabled axis resets the stability window.
    Tick(); Tick(99000);
    CHECK(Gimbal.status == Gimbal_Status_ENABLING);
    Tick(); CHECK(Gimbal.status == Gimbal_Status_READY);
    ins.pitch_rad = 0.3f;
    Tick(100000, true, -1, 1);
    CHECK(Gimbal.status == Gimbal_Status_FAULT);
    Tick(1000000, true, 0, 0); Tick();
    for (int i = 0; i < 100; ++i) Tick();
    CHECK(Gimbal.status == Gimbal_Status_READY);
    CHECK(Near(Gimbal.Target_Pitch_Angle, 0.3f));
}

static void TestTimeout()
{
    Init(); command.mode = GimbalMode::IMU; Publish();
    Tick(1000, true, -1, -1);
    const auto attempts = submit_attempts;
    for (int i = 0; i < 19; ++i) Tick(1000, true, -1, -1);
    CHECK(submit_attempts == attempts);
    Tick(1000, true, -1, -1); CHECK(submit_attempts == attempts + 2);
    Tick(1980000, true, -1, -1);
    CHECK(Gimbal.status == Gimbal_Status_FAULT); Zero();
    Tick(1000000, true, -1, -1);
    CHECK(Gimbal.status == Gimbal_Status_ENABLING);
}
static void TestFailures()
{
    Init(); submit_ok = false;
    command.mode = GimbalMode::IMU; Publish(); Tick(1000, true, 0, 0);
    CHECK(Gimbal.status == Gimbal_Status_ENABLING && discrete.empty());
    const auto attempts = submit_attempts;
    submit_ok = true; Tick(20000, true, 0, 0);
    CHECK(submit_attempts == attempts + 2);
    Tick(); for (int i = 0; i < 100; ++i) Tick();
    CHECK(Gimbal.status == Gimbal_Status_READY);
    perform_ok = false; Tick();
    CHECK(Gimbal.status == Gimbal_Status_FAULT);
    const auto failed = perform_attempts;
    perform_ok = true; Tick(); Zero(); CHECK(perform_attempts > failed);
    command.mode = GimbalMode::DISABLED; Publish(); submit_ok = false; Tick();
    CHECK(Gimbal.status == Gimbal_Status_DISABLE);
    const auto before = submit_attempts;
    submit_ok = true; Tick(20000);
    CHECK(submit_attempts == before + 2 && discrete.back().data[7] == 0xfd);
}
static void TestInvalid()
{
    Init(); Activate();
    command.pitch_angle_rad = std::numeric_limits<float>::quiet_NaN(); Publish(); Tick();
    CHECK(Gimbal.status == Gimbal_Status_FAULT); Zero();
    command.pitch_angle_rad = 0; Publish();
    ins.gyro_y_rad_s = std::numeric_limits<float>::infinity(); Tick(1000000);
    CHECK(Gimbal.status == Gimbal_Status_FAULT); Zero();
    ins.gyro_y_rad_s = 0; Tick(); Tick();
    for (int i = 0; i < 100; ++i) Tick();
    CHECK(Gimbal.status == Gimbal_Status_READY);
    Gimbal.Pitch_Motor.feedback.velocity = std::numeric_limits<float>::quiet_NaN();
    Tick(1000, true, 1, -1);
    CHECK(Gimbal.status == Gimbal_Status_FAULT); Zero();
}
static void TestDriver()
{
    Init(); Receive(cfg.yaw, 1, 1, 2);
    test_irq_mask = 1;
    const auto snapshot = Gimbal.Yaw_Motor.GetFeedbackSnapshot();
    CHECK(test_irq_mask == 1 && snapshot.online && snapshot.enabled);
    CHECK(Near(snapshot.feedback.position, 1)); test_irq_mask = 0;
    test_timestamp_us += 100000;
    CHECK(!Gimbal.Yaw_Motor.GetFeedbackSnapshot().online); // No Daemon CheckAll required.
    perform_ok = false;
    CHECK(!Gimbal.Yaw_Motor.SetTorque(1));
    CHECK(!Gimbal.Pitch_Motor.SetMIT(0, 0, 20, 1, 0));
    perform_ok = true; CHECK(Gimbal.Yaw_Motor.SetTorque(100));
    CHECK(Near(Decode(cfg.yaw).torque, cfg.yaw.torque_max));
}
int main(int argc, char **argv)
{
    CHECK(argc == 2);
    if (!std::strcmp(argv[1], "config")) TestConfig();
    else if (!std::strcmp(argv[1], "init_failure"))
    {
        cfg = Gimbal_Default_Config(); registration_ok = false;
        CHECK(!Gimbal_Init(cfg)); command.mode = GimbalMode::IMU; Publish(); Tick();
        CHECK(Gimbal.status == Gimbal_Status_CONFIG_ERROR && discrete.empty());
    }
    else if (!std::strcmp(argv[1], "partial_init"))
    {
        cfg = Gimbal_Default_Config(); registration_limit = 1;
        CHECK(!Gimbal_Init(cfg)); Tick();
        CHECK(Gimbal.status == Gimbal_Status_CONFIG_ERROR);
        CHECK(discrete.size() == 1 && discrete[0].id == cfg.yaw.id && discrete[0].data[7] == 0xfd);
        CHECK(periodic.size() == 1 && Near(Decode(cfg.yaw).torque, 0));
    }
    else if (!std::strcmp(argv[1], "stable")) TestStable();
    else if (!std::strcmp(argv[1], "control")) TestControl();
    else if (!std::strcmp(argv[1], "pitch")) TestPitch(false);
    else if (!std::strcmp(argv[1], "direction")) TestPitch(true);
    else if (!std::strcmp(argv[1], "modes")) TestModes();
    else if (!std::strcmp(argv[1], "recovery")) TestRecovery();
    else if (!std::strcmp(argv[1], "timeout")) TestTimeout();
    else if (!std::strcmp(argv[1], "failures")) TestFailures();
    else if (!std::strcmp(argv[1], "invalid")) TestInvalid();
    else if (!std::strcmp(argv[1], "driver")) TestDriver();
    else if (!std::strcmp(argv[1], "defaults"))
    {
        Init(false); Activate(); command.yaw_angle_rad = 2; Publish(); Tick();
        CHECK(Near(Decode(cfg.yaw).torque, 0));
    }
    else CHECK(false);
    CHECK(test_irq_mask == 0);
    std::printf("PASS gimbal %s\n", argv[1]);
}

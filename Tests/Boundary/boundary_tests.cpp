#include "alg_pid.h"
#include "alg_filter_kalman.h"
#include "dmmotor.h"
#include "QD4310.h"
#include "dvc_erictool.h"
#include "sys_timestamp.h"
#include <stdio.h>
#include <stdlib.h>
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <sys/mman.h>
#include <unistd.h>
#endif

static unsigned checks;
#define CHECK(condition) do { ++checks; if (!(condition)) { \
    fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition); exit(1); } } while (0)
static bool Near(float actual, float expected)
{
    return isfinite(actual) && fabsf(actual - expected) < 0.0001f;
}

uint32_t test_irq_mask;
uint64_t test_timestamp_us;
Class_Timestamp SYS_Timestamp;
Struct_UART_Manage_Object USART1_Manage_Object{}, USART2_Manage_Object{},
    USART3_Manage_Object{}, UART5_Manage_Object{}, USART6_Manage_Object{},
    UART7_Manage_Object{}, USART10_Manage_Object{};
Struct_USB_Manage_Object USB0_Manage_Object{};
uint8_t UART_Transmit_Data(UART_HandleTypeDef *, uint8_t *, uint16_t) { return 0; }
uint8_t USB_Transmit_Data(uint8_t *, uint16_t) { return 0; }

static bool submit_ok, perform_ok;
static unsigned submit_calls, perform_calls;
static Struct_CAN_Tx_Msg last_message;
static CAN_RxCallback_t rx_callback;
static void *rx_context;
bool BSP_CAN_RegisterCallback(uint32_t, FDCAN_HandleTypeDef *, CAN_RxCallback_t callback, void *context)
{
    rx_callback = callback;
    rx_context = context;
    return true;
}
bool CAN_Tx_Submit(const Struct_CAN_Tx_Msg *message)
{
    ++submit_calls;
    last_message = *message;
    return submit_ok;
}
bool CAN_Tx_Perform(const Struct_CAN_Tx_Msg *message)
{
    ++perform_calls;
    last_message = *message;
    return perform_ok;
}

/** @brief 验证死区目标保持、边界连续性和已有积分/前馈契约。 @author zzm */
static void TestPIDDeadZone()
{
    const float feedback[] = {0.05f, 0.10f, 0.15f};
    const float expected[] = {0.0f, 0.0f, -0.025f};
    for (int reset_target = 0; reset_target < 2; ++reset_target)
    {
        Class_PID pid;
        pid.Init(1, 0, 0, 0, 0, 0, 0.001f, 0.125f);
        pid.Set_Target(0);
        for (int i = 0; i < 3; ++i)
        {
            if (reset_target) pid.Set_Target(0); // DJI 各环每次计算前重设目标。
            pid.Set_Now(feedback[i]);
            pid.TIM_Calculate_PeriodElapsedCallback();
            CHECK(Near(pid.Get_Target(), 0));
            CHECK(Near(pid.Get_Error(), expected[i]));
            CHECK(Near(pid.Get_Out(), expected[i]));
        }
    }

    const float errors[] = {-0.126f, -0.125f, -0.124f, 0.124f, 0.125f, 0.126f};
    const float outputs[] = {-0.001f, 0, 0, 0, 0, 0.001f};
    Class_PID boundary;
    boundary.Init(1, 0, 0, 0, 0, 0, 0.001f, 0.125f);
    for (int i = 0; i < 6; ++i)
    {
        boundary.Set_Target(errors[i]);
        boundary.TIM_Calculate_PeriodElapsedCallback();
        CHECK(Near(boundary.Get_Target(), errors[i]));
        CHECK(Near(boundary.Get_Out(), outputs[i]));
    }

    Class_PID feedforward;
    feedforward.Init(0, 0, 0, 1, 0, 0, 0.001f, 0.125f);
    for (int i = 0; i < 3; ++i)
    {
        feedforward.Set_Target(0);
        feedforward.Set_Now(feedback[i]);
        feedforward.TIM_Calculate_PeriodElapsedCallback();
        CHECK(Near(feedforward.Get_Out(), 0));
    }
    feedforward.Set_Target(0.25f);
    feedforward.Set_Now(0.30f);
    feedforward.TIM_Calculate_PeriodElapsedCallback();
    CHECK(Near(feedforward.Get_Target(), 0.25f));
    CHECK(Near(feedforward.Get_Out(), 0.25f)); // 真实目标增量前馈，不除以 dt。
    feedforward.TIM_Calculate_PeriodElapsedCallback();
    CHECK(Near(feedforward.Get_Out(), 0));

    Class_PID integral;
    integral.Init(0, 1, 0, 0, 5, 0, 0.001f, 0.125f);
    integral.Set_Integral_Error(2);
    integral.Set_Now(0.05f);
    integral.TIM_Calculate_PeriodElapsedCallback();
    CHECK(Near(integral.Get_Error(), 0));
    CHECK(Near(integral.Get_Out(), 2)); // 死区不清除已有负载补偿。

    Class_PID derivative;
    derivative.Init(0, 0, 1, 0, 0, 0, 0.001f, 0.125f, 0, 0, 0, PID_D_First_ENABLE);
    derivative.Set_Now(0.05f);
    derivative.TIM_Calculate_PeriodElapsedCallback();
    CHECK(Near(derivative.Get_Error(), 0));
    CHECK(Near(derivative.Get_Out(), -50)); // 测量微分不受有效误差归零影响。

    Class_PID variable;
    variable.Init(0, 1, 0, 0, 0, 0, 0.001f, 0.125f, 0.2f, 0.4f);
    variable.Set_Target(0.3f);
    variable.TIM_Calculate_PeriodElapsedCallback();
    CHECK(Near(variable.Get_Error(), 0.175f));
    CHECK(Near(variable.Get_Integral_Error() * 1000, 0.0875f)); // 区外阈值仍使用原始误差幅值。
    variable.Set_I_Separate_Threshold(0.2f);
    variable.Set_Integral_Error(2);
    variable.TIM_Calculate_PeriodElapsedCallback();
    CHECK(Near(variable.Get_Integral_Error(), 0)); // 分离仍清空，不改成冻结。

    Class_PID error_derivative;
    error_derivative.Init(0, 0, 1, 0, 0, 0, 0.001f, 0.125f);
    error_derivative.Set_Target(0.124f);
    error_derivative.TIM_Calculate_PeriodElapsedCallback();
    CHECK(Near(error_derivative.Get_Out(), 0));
    error_derivative.Set_Target(0.125f);
    error_derivative.TIM_Calculate_PeriodElapsedCallback();
    CHECK(Near(error_derivative.Get_Out(), 0));
    error_derivative.Set_Target(0.126f);
    error_derivative.TIM_Calculate_PeriodElapsedCallback();
    CHECK(Near(error_derivative.Get_Out(), 1));
}

/** @brief 验证生产 IIR 在 D 支路的响应、旁路、增益和参数更新语义。 @author zzm */
static void TestPIDDerivativeFilter()
{
    const float dt = 0.001f;
    const float cutoff = 50.0f;
    const float alpha = 1.0f - expf(-2.0f * PI * cutoff * dt);
    const PID_InitTypeDef legacy_config = {0, 0, 1, 0, 0, 0, dt, 0, 0, 0, 0, PID_D_First_DISABLE};
    PID_InitTypeDef assigned_config; // 旧式逐字段赋值也必须默认关闭新增选项。
    CHECK(Near(legacy_config.D_Filter_Cutoff, 0));
    CHECK(Near(assigned_config.D_Filter_Cutoff, 0));

    for (int mode = 0; mode < 2; ++mode)
    {
        const Enum_PID_D_First d_first = (Enum_PID_D_First)mode;
        const float sign = mode == 0 ? 1.0f : -1.0f;
        Class_PID raw, filtered;
        raw.Init(0, 0, 1, 0, 0, 0, dt, 0, 0, 0, 0, d_first);
        filtered.Init(0, 0, 1, 0, 0, 0, dt, 0, 0, 0, 0, d_first, cutoff);
        if (mode == 0)
        {
            raw.Set_Target(1);
            filtered.Set_Target(1);
        }
        else
        {
            raw.Set_Now(1);
            filtered.Set_Now(1);
        }
        raw.TIM_Calculate_PeriodElapsedCallback();
        filtered.TIM_Calculate_PeriodElapsedCallback();
        CHECK(Near(raw.Get_Out() * dt, sign));
        CHECK(Near(filtered.Get_Out() * dt, sign * alpha)); // 首帧也滤波，不直通尖峰。
        float expected = sign * alpha;
        for (int i = 0; i < 8; ++i)
        {
            raw.TIM_Calculate_PeriodElapsedCallback();
            filtered.TIM_Calculate_PeriodElapsedCallback();
            expected *= 1.0f - alpha;
            CHECK(Near(raw.Get_Out(), 0));
            CHECK(Near(filtered.Get_Out() * dt, expected));
        }
    }

    Class_PID measurement;
    measurement.Init(0, 0, 1, 0, 0, 0, dt, 0, 0, 0, 0, PID_D_First_ENABLE, cutoff);
    measurement.Set_Target(10);
    measurement.TIM_Calculate_PeriodElapsedCallback();
    CHECK(Near(measurement.Get_Out(), 0)); // D_First 的目标阶跃抑制仍有效。
    measurement.Set_Now(10);
    measurement.TIM_Calculate_PeriodElapsedCallback();
    CHECK(Near(measurement.Get_Out() * dt, -10.0f * alpha)); // 低通未冒充历史对齐。

    Class_PID gain;
    gain.Init(0, 0, 1, 0, 0, 0, dt, 0, 0, 0, 0, PID_D_First_DISABLE, cutoff);
    gain.Set_Target(1);
    gain.TIM_Calculate_PeriodElapsedCallback();
    gain.Set_K_D(0);
    gain.Set_Target(2);
    gain.TIM_Calculate_PeriodElapsedCallback();
    CHECK(Near(gain.Get_Out(), 0)); // Kd=0 时没有旧 D 输出尾巴。
    const float rate2 = alpha + (1.0f - alpha) * alpha;
    gain.Set_K_D(-2);
    gain.Set_Target(3);
    gain.TIM_Calculate_PeriodElapsedCallback();
    CHECK(Near(gain.Get_Out() * dt, -2.0f * (alpha + (1.0f - alpha) * rate2)));

    Class_PID update;
    update.Init(0, 1, 1, 0, 0, 0, dt, 0, 0, 0, 0, PID_D_First_DISABLE, cutoff);
    update.Set_Target(1);
    update.TIM_Calculate_PeriodElapsedCallback();
    const float old_out = update.Get_Out();
    const float old_integral = update.Get_Integral_Error();
    update.Init(0, 1, 1, 0, 0, 0, 2 * dt, 0, 0, 0, 0, PID_D_First_DISABLE, 2 * cutoff);
    CHECK(Near(update.Get_Target(), 1));
    CHECK(Near(update.Get_Out(), old_out));
    CHECK(Near(update.Get_Integral_Error(), old_integral));
    update.TIM_Calculate_PeriodElapsedCallback();
    const float new_alpha = 1.0f - expf(-2.0f * PI * (2 * cutoff) * (2 * dt));
    CHECK(Near((update.Get_Out() - update.Get_Integral_Error()) * dt, alpha * (1.0f - new_alpha)));
    // 关闭立即旁路，再启用不会恢复之前的滤波尾巴。
    update.Init(0, 0, 1, 0, 0, 0, dt);
    update.TIM_Calculate_PeriodElapsedCallback();
    CHECK(Near(update.Get_Out(), 0));
    update.Init(0, 0, 1, 0, 0, 0, dt, 0, 0, 0, 0, PID_D_First_DISABLE, cutoff);
    update.TIM_Calculate_PeriodElapsedCallback();
    CHECK(Near(update.Get_Out(), 0));
    update.Set_Target(2);
    update.TIM_Calculate_PeriodElapsedCallback();
    update.Init(0, 0, 1, 0, 0, 0, dt, 0, 0, 0, 0, PID_D_First_ENABLE, cutoff);
    update.TIM_Calculate_PeriodElapsedCallback();
    CHECK(Near(update.Get_Out(), 0)); // 切换微分来源不继承误差微分的滤波尾巴。

    // 只有 D 支路经过滤波，P/I/F 以及末端 Out_Max 不改变位置。
    Class_PID combined;
    combined.Init(2, 3, 1, 4, 0, 0, dt, 0, 0, 0, 0, PID_D_First_DISABLE, cutoff);
    combined.Set_Target(1);
    combined.TIM_Calculate_PeriodElapsedCallback();
    CHECK(Near((combined.Get_Out() - 2.0f - 3.0f * dt - 4.0f) * dt, alpha));
    combined.Set_Out_Max(10);
    combined.TIM_Calculate_PeriodElapsedCallback();
    CHECK(Near(combined.Get_Out(), 10));

    // 高频反馈扰动下比较实际生产 PID 响应，检验抑制量而非只验证接线。
    Class_PID noisy;
    noisy.Init(0, 0, 1, 0, 0, 0, dt, 0, 0, 0, 0, PID_D_First_ENABLE, cutoff);
    for (int i = 0; i < 100; ++i)
    {
        noisy.Set_Now(i % 2 == 0 ? 1.0f : -1.0f);
        noisy.TIM_Calculate_PeriodElapsedCallback();
    }
    CHECK(Near(fabsf(noisy.Get_Out()) * dt / 2.0f, alpha / (2.0f - alpha)));

    const float bypass_cutoffs[] = {0, -1, NAN, INFINITY};
    for (float bypass_cutoff : bypass_cutoffs)
    {
        Class_PID bypass;
        bypass.Init(0, 0, 1, 0, 0, 0, dt, 0, 0, 0, 0, PID_D_First_DISABLE, bypass_cutoff);
        bypass.Set_Target(1);
        bypass.TIM_Calculate_PeriodElapsedCallback();
        CHECK(Near(bypass.Get_D_Filter_Cutoff(), 0));
        CHECK(Near(bypass.Get_Out() * dt, 1));
    }
}

static void TestPID()
{
    Class_PID pid;
    pid.Init(0, 1, 0, 0, 1);
    pid.Set_Target(1000);
    for (int i = 0; i < 4; ++i)
    {
        pid.TIM_Calculate_PeriodElapsedCallback();
        CHECK(Near(pid.Get_Out(), 1));
        CHECK(Near(pid.Get_Integral_Error(), 1));
    }
    pid.Set_Target(-10000);
    pid.TIM_Calculate_PeriodElapsedCallback();
    CHECK(Near(pid.Get_Out(), -1));
    pid.Set_K_I(-2);
    pid.Set_Target(10000);
    pid.TIM_Calculate_PeriodElapsedCallback();
    CHECK(Near(pid.Get_Out(), -1));
    CHECK(Near(pid.Get_Integral_Error(), 0.5f));

    pid.Set_K_I(0);
    pid.Set_Integral_Error(42);
    pid.TIM_Calculate_PeriodElapsedCallback();
    CHECK(Near(pid.Get_Out(), 0));
    CHECK(Near(pid.Get_Integral_Error(), 0));
    pid.Set_K_I(1);
    pid.Set_Target(1);
    pid.TIM_Calculate_PeriodElapsedCallback();
    CHECK(Near(pid.Get_Out(), 0.001f));

    pid.Set_Integral_Error(0);
    pid.Set_I_Out_Max(0);
    pid.Set_Target(1000);
    for (int i = 1; i <= 3; ++i)
    {
        pid.TIM_Calculate_PeriodElapsedCallback();
        CHECK(Near(pid.Get_Out(), (float)i));
    }
    pid.Set_I_Separate_Threshold(10);
    pid.Set_Target(10);
    pid.TIM_Calculate_PeriodElapsedCallback();
    CHECK(Near(pid.Get_Integral_Error(), 0));
    pid.Set_Target(9);
    pid.TIM_Calculate_PeriodElapsedCallback();
    CHECK(Near(pid.Get_Out(), 0.009f));

    pid.Set_Integral_Error(0);
    pid.Set_I_Variable_Speed_A(2);
    pid.Set_I_Variable_Speed_B(4);
    pid.Set_Target(3);
    pid.TIM_Calculate_PeriodElapsedCallback();
    CHECK(Near(pid.Get_Out(), 0.0015f));
    pid.Set_K_P(5);
    pid.Set_Out_Max(2);
    pid.TIM_Calculate_PeriodElapsedCallback();
    CHECK(Near(pid.Get_Out(), 2));
    TestPIDDeadZone();
    TestPIDDerivativeFilter();
}

static void TestKalman()
{
    Class_Matrix_f32<1, 1> one, zero;
    one.Data[0] = 1;
    Class_Filter_Kalman<1, 1, 1> kf;
    kf.Init(one, one, one, one, one, one, zero, one);
    for (int i = 1; i <= 3; ++i)
    {
        kf.TIM_Predict_PeriodElapsedCallback();
        CHECK(Near(kf.Vector_X.Data[0], (float)i));
        CHECK(Near(kf.Vector_X_Prior.Data[0], (float)i));
        CHECK(Near(kf.Matrix_P.Data[0], (float)(i + 1)));
        CHECK(Near(kf.Matrix_P_Prior.Data[0], (float)(i + 1)));
    }
    kf.Vector_Z.Data[0] = 5;
    kf.TIM_Update_PeriodElapsedCallback();
    CHECK(Near(kf.Matrix_K.Data[0], 0.8f));
    CHECK(Near(kf.Vector_X.Data[0], 4.6f));
    CHECK(Near(kf.Matrix_P.Data[0], 0.8f));
    kf.TIM_Predict_PeriodElapsedCallback();
    CHECK(Near(kf.Vector_X.Data[0], 5.6f));
    CHECK(Near(kf.Matrix_P.Data[0], 1.8f));

    kf.Init(one, one, one, one, one, one, zero, one);
    kf.TIM_Predict_PeriodElapsedCallback();
    kf.Vector_Z.Data[0] = 4;
    kf.TIM_Update_PeriodElapsedCallback();
    CHECK(Near(kf.Vector_X.Data[0], 3));
    CHECK(Near(kf.Matrix_P.Data[0], 2.0f / 3.0f));

    Class_Filter_Kalman<2, 1, 1> motion;
    motion.Matrix_A.Data[0] = 1;
    motion.Matrix_A.Data[1] = 1;
    motion.Matrix_A.Data[3] = 1;
    motion.Matrix_P.Data[0] = motion.Matrix_P.Data[3] = 1;
    motion.Matrix_Q.Data[0] = 0.1f;
    motion.Matrix_Q.Data[3] = 0.2f;
    motion.Vector_X.Data[1] = 2;
    motion.TIM_Predict_PeriodElapsedCallback();
    motion.TIM_Predict_PeriodElapsedCallback();
    CHECK(Near(motion.Vector_X.Data[0], 4));
    CHECK(Near(motion.Vector_X.Data[1], 2));
    CHECK(Near(motion.Matrix_P.Data[0], 5.4f));
    CHECK(Near(motion.Matrix_P.Data[1], 2.2f));
    CHECK(Near(motion.Matrix_P.Data[2], 2.2f));
    CHECK(Near(motion.Matrix_P.Data[3], 1.4f));
}

static FDCAN_HandleTypeDef bus{1};
static void PositionFeedback(uint16_t position)
{
    uint8_t frame[8] = {0x11, (uint8_t)(position >> 8), (uint8_t)position, 0, 0, 0, 20, 21};
    rx_callback(&bus, 0x101, frame, 8, rx_context);
}
static void ModeFeedback(Enum_DMMotor_Mode mode)
{
    uint8_t frame[8] = {1, 0, 0x55, 0x0A, (uint8_t)mode, 0, 0, 0};
    rx_callback(&bus, 0x101, frame, 8, rx_context);
}

static void TestCommands()
{
    Class_DMMotor motor;
    CHECK(motor.Init(&bus, 1, 0x101, Enum_DMMotor_Mode::MIT));
    for (int accepted = 0; accepted <= 1; ++accepted)
    {
        submit_ok = accepted != 0;
        CHECK(motor.Enable() == submit_ok);
        CHECK(last_message.id == 1 && last_message.len == 8 && last_message.data[7] == 0xFC);
        for (int i = 0; i < 7; ++i) CHECK(last_message.data[i] == 0xFF);
        CHECK(motor.Disable() == submit_ok);
        CHECK(last_message.data[7] == 0xFD);
        CHECK(motor.ClearError() == submit_ok);
        CHECK(last_message.data[7] == 0xFB);
    }
    PositionFeedback(65000);
    PositionFeedback(0);
    const float before_zero = motor.feedback.total_position;
    CHECK(before_zero > 12);
    submit_ok = false;
    CHECK(!motor.SetZeroPosition());
    CHECK(motor.feedback.total_position == before_zero);
    PositionFeedback(1000);
    CHECK(motor.feedback.total_position > 12);
    submit_ok = true;
    CHECK(motor.SetZeroPosition());
    CHECK(last_message.data[7] == 0xFE);
    CHECK(Near(motor.feedback.total_position, 0));
    PositionFeedback(1000);
    CHECK(motor.feedback.total_position < -12);

    unsigned calls = submit_calls;
    CHECK(!motor.SetMode((Enum_DMMotor_Mode)0));
    CHECK(!motor.SetMode((Enum_DMMotor_Mode)5));
    CHECK(submit_calls == calls);
    submit_ok = false;
    CHECK(!motor.SetMode(Enum_DMMotor_Mode::SPEED));
    CHECK(test_irq_mask == 0);
    submit_ok = true;
    CHECK(motor.SetMode(Enum_DMMotor_Mode::POSITION_SPEED));
    CHECK(last_message.id == 0x7FF && last_message.data[4] == 2);
    CHECK(motor.Enable());
    CHECK(last_message.id == 1); // No optimistic mode change before acknowledgement.
    calls = submit_calls;
    CHECK(!motor.SetMode(Enum_DMMotor_Mode::SPEED));
    CHECK(submit_calls == calls);
    test_timestamp_us = 100000;
    submit_ok = false;
    test_irq_mask = 1;
    CHECK(!motor.SetMode(Enum_DMMotor_Mode::POSITION_SPEED));
    CHECK(test_irq_mask == 1);
    test_irq_mask = 0;
    submit_ok = true;
    CHECK(motor.SetMode(Enum_DMMotor_Mode::POSITION_SPEED));
    test_timestamp_us = 250000; // Retry must not extend the first request's deadline.
    CHECK(motor.SetMode(Enum_DMMotor_Mode::SPEED));
    ModeFeedback(Enum_DMMotor_Mode::POSITION_SPEED);
    CHECK(motor.Enable());
    CHECK(last_message.id == 1);
    ModeFeedback(Enum_DMMotor_Mode::SPEED);
    CHECK(motor.Enable());
    CHECK(last_message.id == 0x201);

    QD4310_t qd{};
    QD4310_Init(&qd, 2, &bus);
    for (int accepted = 0; accepted <= 1; ++accepted)
    {
        submit_ok = accepted != 0;
        perform_ok = !submit_ok;
        const unsigned periodic_before = perform_calls;
        CHECK(QD4310_Enable(&qd) == submit_ok);
        CHECK(last_message.hfdcan == &bus && last_message.id == 0x402 && last_message.len == 3);
        CHECK(last_message.data[0] == QD4310_CMD_ENABLE);
        CHECK(QD4310_Disable(&qd) == submit_ok);
        CHECK(QD4310_SetStepAngle(&qd, 0) == submit_ok);
        CHECK(QD4310_SetZeroAngle(&qd) == submit_ok);
        CHECK(QD4310_SendCommand(&qd, QD4310_CMD_CLEAR_ERROR, 0) == submit_ok);
        CHECK(perform_calls == periodic_before);
        calls = submit_calls;
        CHECK(QD4310_SetCurrent(&qd, -10) == perform_ok);
        CHECK(last_message.data[0] == QD4310_CMD_CURRENT);
        CHECK(last_message.data[1] == 0x01 && last_message.data[2] == 0x80);
        CHECK(QD4310_SetSpeed(&qd, 0) == perform_ok);
        CHECK(QD4310_SetLowSpeed(&qd, 0) == perform_ok);
        CHECK(QD4310_SetAngle(&qd, 0) == perform_ok);
        CHECK(submit_calls == calls && perform_calls == periodic_before + 4);
        CHECK(!qd.enabled); // Submission never fabricates motor feedback.
    }
}

static Class_EricTool_UART uart;
static Class_EricTool_USB usb;
static uint8_t *guarded_memory;
static size_t page_size;
static void CheckParse(const char *text, uint16_t length, int32_t index, float value)
{
    // Every input ends exactly at an inaccessible page; even a one-byte overread faults.
    CHECK(length <= page_size);
    uint8_t *data = guarded_memory + page_size - length;
    if (length != 0) memcpy(data, text, length);
    uart.UART_RxCpltCallback(data, length);
    usb.USB_RxCallback(data, length);
    CHECK(uart.Get_Variable_Index() == index);
    CHECK(usb.Get_Variable_Index() == index);
    CHECK(Near(uart.Get_Variable_Value(), value));
    CHECK(Near(usb.Get_Variable_Value(), value));
}

static void TestParser()
{
#ifdef _WIN32
    SYSTEM_INFO info;
    GetSystemInfo(&info);
    page_size = info.dwPageSize;
    guarded_memory = (uint8_t *)VirtualAlloc(nullptr, page_size * 2, MEM_RESERVE | MEM_COMMIT, PAGE_READWRITE);
    CHECK(guarded_memory != nullptr);
    DWORD old_protection;
    CHECK(VirtualProtect(guarded_memory + page_size, page_size, PAGE_NOACCESS, &old_protection) != 0);
#else
    page_size = (size_t)sysconf(_SC_PAGESIZE);
    guarded_memory = (uint8_t *)mmap(nullptr, page_size * 2, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    CHECK(guarded_memory != MAP_FAILED);
    CHECK(mprotect(guarded_memory + page_size, page_size, PROT_NONE) == 0);
#endif
    UART_HandleTypeDef handle{USART1};
    char long_name[100];
    memset(long_name, 'a', 99);
    long_name[99] = 0;
    const char *names[] = {"x", "gain", nullptr, long_name};
    uart.Init(&handle, 4, names);
    usb.Init(4, names);
    CHECK(uart.Get_Variable_Index() == -1 && usb.Get_Variable_Index() == -1);
    CheckParse("x:1#", 4, 0, 1);
    CheckParse("gain:-12.5#", 11, 1, -12.5f);
    CheckParse("x:.5#", 5, 0, 0.5f);
    CheckParse("x:1.#", 5, 0, 1);
    CheckParse("x:0#tail", 8, 0, 0); // Preserve first-frame behavior.
    const char *invalid[] = {"", "x", "x:", "x:1", "x:#", "x:-#", "x:.#", "x:-.#",
        "x:1..2#", "x:1a#", "x:--1#", "x:+1#", "x:1e2#", "x: 1#", ":1#", "z:1#", "x=1#"};
    for (const char *text : invalid)
    {
        CheckParse("x:9#", 4, 0, 9);
        CheckParse(text, (uint16_t)strlen(text), -1, 0);
    }
    const char *valid = "gain:-12.5#";
    for (uint16_t length = 0; length < strlen(valid); ++length)
        CheckParse(valid, length, -1, 0);
    CheckParse("x:1\0#", 5, -1, 0);
    CheckParse("x\0:1#", 5, -1, 0);
    char boundary[104];
    memset(boundary, 'a', 100);
    memcpy(boundary + 99, ":1#", 3);
    CheckParse(boundary, 102, 3, 1);
    boundary[99] = 'a';
    memcpy(boundary + 100, ":1#", 3);
    CheckParse(boundary, 103, -1, 0);
    memset(boundary, '9', sizeof(boundary));
    boundary[0] = 'x'; boundary[1] = ':'; boundary[103] = '#';
    CheckParse(boundary, sizeof(boundary), -1, 0); // Numeric overflow.
    uart.UART_RxCpltCallback(nullptr, 1);
    usb.USB_RxCallback(nullptr, 1);
    CHECK(uart.Get_Variable_Index() == -1 && usb.Get_Variable_Index() == -1);
    uart.Init(&handle);
    usb.Init();
    CheckParse("x:1#", 4, -1, 0);
#ifdef _WIN32
    CHECK(VirtualFree(guarded_memory, 0, MEM_RELEASE) != 0);
#else
    CHECK(munmap(guarded_memory, page_size * 2) == 0);
#endif
}

int main(int argc, char **argv)
{
    if (argc != 2) return 2;
    if (strcmp(argv[1], "pid") == 0) TestPID();
    else if (strcmp(argv[1], "kalman") == 0) TestKalman();
    else if (strcmp(argv[1], "commands") == 0) TestCommands();
    else if (strcmp(argv[1], "parser") == 0) TestParser();
    else return 2;
    printf("PASS %s: %u checks\n", argv[1], checks);
    return 0;
}

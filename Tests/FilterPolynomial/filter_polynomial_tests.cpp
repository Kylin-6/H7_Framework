#include "alg_filter_polynomial.h"
#include "reference_coefficients.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned long long checks = 0;
static uint32_t random_state = 73921;
static unsigned current_order = 0;
static unsigned current_window = 0;

static void Check(bool condition, const char *message)
{
    checks++;
    if (!condition)
    {
        fprintf(stderr, "FAIL order=%u window=%u: %s\n",
                current_order, current_window, message);
        exit(1);
    }
}

static void Check_Close(double actual, double expected, double tolerance,
                        const char *message)
{
    if (!isfinite(actual) || fabs(actual - expected) > tolerance)
    {
        fprintf(stderr, "actual=%.17g expected=%.17g tolerance=%.17g\n",
                actual, expected, tolerance);
        Check(false, message);
    }
    Check(true, message);
}

static float Random()
{
    random_state ^= random_state << 13;
    random_state ^= random_state >> 17;
    random_state ^= random_state << 5;
    return (float)(random_state / 2147483648.0 - 1.0);
}

class Test_Filter : public Class_Filter_Polynomial
{
public:
    float Get_Coefficient(unsigned derivative, unsigned index) const
    {
        return Coefficient[derivative][index];
    }

    bool Matches(const Test_Filter *other) const
    {
        return memcmp(Coefficient, other->Coefficient, sizeof(Coefficient)) == 0 &&
               memcmp(Input_Signal, other->Input_Signal, sizeof(Input_Signal)) == 0 &&
               memcmp(Out, other->Out, sizeof(Out)) == 0 && Now == other->Now &&
               Window_Size == other->Window_Size && Polynomial_Order == other->Polynomial_Order &&
               Signal_Flag == other->Signal_Flag && Sample_Count == other->Sample_Count;
    }
};

static float Read_Out(const Class_Filter_Polynomial *filter, unsigned derivative)
{
    switch (derivative)
    {
    case 0: return filter->Get_Out();
    case 1: return filter->Get_First_Derivative();
    case 2: return filter->Get_Second_Derivative();
    default: return filter->Get_Third_Derivative();
    }
}

static void Step(Class_Filter_Polynomial *filter, float value)
{
    filter->Set_Now(value);
    filter->TIM_Calculate_PeriodElapsedCallback();
}

static unsigned Reference_Count()
{
    return sizeof(POLYNOMIAL_REFERENCES) / sizeof(POLYNOMIAL_REFERENCES[0]);
}

static void Test_Reference()
{
    const float periods[] = {1.0f, 0.0001f, 0.001f, 0.02f};
    for (unsigned r = 0; r < Reference_Count(); r++)
    {
        const Struct_Polynomial_Reference *reference = &POLYNOMIAL_REFERENCES[r];
        current_order = reference->Order;
        current_window = reference->Window_Size;
        for (unsigned p = 0; p < sizeof(periods) / sizeof(periods[0]); p++)
        {
            Test_Filter filter;
            Check(filter.Init(current_window, periods[p], current_order), "valid configuration");
            double scale = 1.0;
            for (unsigned derivative = 0; derivative < 4; derivative++)
            {
                double maximum = 0.0;
                for (unsigned i = 0; i < current_window; i++)
                {
                    double value = fabs(reference->Coefficient[derivative][i] * scale);
                    if (value > maximum) maximum = value;
                }
                for (unsigned i = 0; i < FILTER_POLYNOMIAL_MAX_WINDOW_SIZE; i++)
                {
                    double expected = reference->Coefficient[derivative][i] * scale;
                    Check_Close(filter.Get_Coefficient(derivative, i), expected,
                                2 * FLT_EPSILON * maximum, "independent SciPy coefficients");
                }
                scale /= periods[p];
            }
        }
    }
}

static void Test_Stream()
{
    const float periods[] = {0.0005f, 0.001f, 0.02f};
    for (unsigned r = 0; r < Reference_Count(); r++)
    {
        const Struct_Polynomial_Reference *reference = &POLYNOMIAL_REFERENCES[r];
        current_order = reference->Order;
        current_window = reference->Window_Size;
        for (unsigned p = 0; p < sizeof(periods) / sizeof(periods[0]); p++)
        {
            Class_Filter_Polynomial filter;
            Check(filter.Init(current_window, periods[p], current_order), "stream configuration");
            float history[33] = {};
            for (unsigned sample = 0; sample < 12 * current_window + 29; sample++)
            {
                for (unsigned i = 1; i < current_window; i++) history[i - 1] = history[i];
                float value = Random() + (sample % 17 == 0 ? 2.0f : 0.0f);
                history[current_window - 1] = value;
                Step(&filter, value);
                Check(filter.Get_Ready_Flag() == (sample + 1 >= current_window), "window readiness");
                if (!filter.Get_Ready_Flag())
                {
                    Check(filter.Get_Out() == value, "warmup passthrough");
                    for (unsigned d = 1; d < 4; d++) Check(Read_Out(&filter, d) == 0, "warmup derivative");
                    continue;
                }
                double scale = 1.0;
                for (unsigned d = 0; d < 4; d++)
                {
                    double expected = 0.0;
                    double magnitude = 0.0;
                    for (unsigned i = 0; i < current_window; i++)
                    {
                        double coefficient = reference->Coefficient[d][i] * scale;
                        expected += coefficient * history[i];
                        magnitude += fabs(coefficient) * (fabs(history[i]) + fabs(value));
                    }
                    double tolerance = 8 * FLT_EPSILON * (current_window + 2) * magnitude;
                    if (d == 0) tolerance += 2 * FLT_EPSILON * fabs(value);
                    Check_Close(Read_Out(&filter, d), expected, tolerance, "stream against direct reference convolution");
                    if (d > current_order) Check(Read_Out(&filter, d) == 0, "derivative above model degree");
                    scale /= periods[p];
                }
            }
        }
    }
}

static double Polynomial(double time, unsigned degree, unsigned derivative)
{
    const double coefficients[] = {0.75, -0.125, 0.0625, -0.015625};
    double result = 0.0;
    for (unsigned term = derivative; term <= degree; term++)
    {
        double value = coefficients[term];
        for (unsigned d = 0; d < derivative; d++) value *= term - d;
        for (unsigned power = 0; power < term - derivative; power++) value *= time;
        result += value;
    }
    return result;
}

static void Test_Polynomial()
{
    const float dt = 0.02f;
    for (unsigned r = 0; r < Reference_Count(); r++)
    {
        const Struct_Polynomial_Reference *reference = &POLYNOMIAL_REFERENCES[r];
        current_order = reference->Order;
        current_window = reference->Window_Size;
        for (unsigned degree = 0; degree <= current_order; degree++)
        {
            Class_Filter_Polynomial filter;
            Check(filter.Init(current_window, dt, current_order), "polynomial configuration");
            for (unsigned sample = 0; sample < 4 * current_window + 7; sample++)
            {
                double time = ((double)sample - current_window) * dt;
                Step(&filter, (float)Polynomial(time, degree, 0));
                if (!filter.Get_Ready_Flag()) continue;
                double scale = 1.0;
                for (unsigned d = 0; d < 4; d++)
                {
                    double magnitude = 0.0;
                    for (unsigned i = 0; i < current_window; i++)
                    {
                        double t = time + ((double)i - current_window + 1) * dt;
                        magnitude += fabs(reference->Coefficient[d][i] * scale) *
                                     (fabs(Polynomial(t, degree, 0)) + fabs(Polynomial(time, degree, 0)));
                    }
                    double tolerance = 8 * FLT_EPSILON * (current_window + 2) * magnitude;
                    if (d == 0) tolerance += 2 * FLT_EPSILON * fabs(Polynomial(time, degree, 0));
                    Check_Close(Read_Out(&filter, d), Polynomial(time, degree, d), tolerance,
                                "analytic polynomial and time derivatives");
                    scale /= dt;
                }
            }
        }

        Class_Filter_Polynomial filter;
        Check(filter.Init(current_window, 0.001f, current_order), "large DC configuration");
        for (unsigned sample = 0; sample < 3 * current_window; sample++) Step(&filter, FLT_MAX);
        Check(filter.Get_Out() == FLT_MAX, "constant large DC remains exact");
        for (unsigned d = 1; d < 4; d++) Check(Read_Out(&filter, d) == 0, "constant signal has zero derivatives");
    }
}

static void Test_Lifecycle()
{
    Class_Filter_Polynomial filter;
    Step(&filter, 12.0f);
    Check(!filter.Get_Ready_Flag() && filter.Get_Out() == 0, "unconfigured update is inactive");
    Check(filter.Init(9), "default quadratic configuration");
    Check(filter.Get_Polynomial_Order() == 2, "default degree is two");
    filter.Reset(42.0f);
    Check(filter.Get_Out() == 42.0f && !filter.Get_Ready_Flag(), "reset seeds output only");
    for (unsigned i = 0; i < 8; i++)
    {
        filter.Set_Now(-1000.0f);
        Step(&filter, (float)i);
        Check(!filter.Get_Ready_Flag() && filter.Get_Out() == (float)i, "setter does not create samples");
    }
    Step(&filter, 8.0f);
    Check(filter.Get_Ready_Flag(), "ready after nine actual updates");
    Check_Close(filter.Get_Out(), 8, 1e-5, "linear endpoint after warmup");
    Check_Close(filter.Get_First_Derivative(), 1000, 0.002, "linear slope after warmup");

    Check(filter.Init(4, 0.02f, 3), "reconfigure cubic");
    Check(!filter.Get_Ready_Flag() && filter.Get_Out() == 0, "successful reconfigure clears output and readiness");
    for (unsigned i = 0; i < 12; i++) Step(&filter, (float)(i * i * i));
    Check(filter.Get_Third_Derivative() != 0, "cubic third derivative is exposed");
    filter.Reset(-7.0f);
    Check(filter.Get_Polynomial_Order() == 3 && !filter.Get_Ready_Flag(), "reset retains configuration");
    Check(filter.Get_Out() == -7.0f, "reset scalar output");
    for (unsigned d = 1; d < 4; d++) Check(Read_Out(&filter, d) == 0, "reset clears derivative outputs");
    for (unsigned i = 0; i < 3; i++) Step(&filter, 10.0f);
    Check(!filter.Get_Ready_Flag(), "old history cannot satisfy reset window");
    Step(&filter, 10.0f);
    Check(filter.Get_Ready_Flag() && filter.Get_Out() == 10.0f, "new window replaces old history");

    Check(filter.Init(1, 0.001f, 0), "one-point zero-degree configuration");
    Step(&filter, -15.0f);
    Check(filter.Get_Ready_Flag() && filter.Get_Out() == -15.0f, "zero-degree one-point identity");
    for (unsigned d = 1; d < 4; d++) Check(Read_Out(&filter, d) == 0, "lower degree clears high derivatives");
    Check(filter.Init(4, 1.0f, 0), "moving-average configuration");
    for (unsigned i = 1; i <= 4; i++) Step(&filter, (float)(2 * i));
    Check(filter.Get_Out() == 5.0f, "zero degree is moving average");
    Step(&filter, 10.0f);
    Check(filter.Get_Out() == 7.0f, "moving average rolls forward");
}

static void Test_Boundary()
{
    Test_Filter filter;
    Check(filter.Init(17, 0.001f, 3), "boundary baseline");
    for (unsigned i = 0; i < 31; i++) Step(&filter, Random());
    filter.Set_Now(0.375f);
    const Test_Filter snapshot = filter;
    const unsigned invalid_windows[] = {0, 1, 2, 3, 34, UINT32_MAX};
    for (unsigned i = 0; i < sizeof(invalid_windows) / sizeof(invalid_windows[0]); i++)
    {
        Check(!filter.Init(invalid_windows[i], 0.001f, 3), "reject invalid window");
        Check(filter.Matches(&snapshot), "invalid window preserves full state");
    }
    const float invalid_periods[] = {0.0f, -1.0f, NAN, INFINITY, -INFINITY,
                                     FLT_MIN / 2, FLT_MIN, 1e-20f, 1e20f, FLT_MAX};
    for (unsigned i = 0; i < sizeof(invalid_periods) / sizeof(invalid_periods[0]); i++)
    {
        Check(!filter.Init(17, invalid_periods[i], 3), "reject invalid or unrepresentable period");
        Check(filter.Matches(&snapshot), "failed coefficients preserve full state");
    }
    Check(!filter.Init(17, 0.001f, 4), "reject fourth degree");
    Check(filter.Matches(&snapshot), "invalid degree preserves full state");
    Check(!filter.Init(17, 0.001f, UINT32_MAX), "reject degree overflow");
    Check(filter.Matches(&snapshot), "degree overflow preserves full state");
    Test_Filter control = snapshot;
    filter.TIM_Calculate_PeriodElapsedCallback();
    control.TIM_Calculate_PeriodElapsedCallback();
    Check(filter.Matches(&control), "pending sample survives failed reinitialization");

    Check(filter.Init(1, FLT_MIN, 0), "zero degree does not require representable derivatives");
    Check(filter.Init(33, FLT_MAX, 0), "zero degree accepts large finite period");
}

int main(int argc, char **argv)
{
    Check(argc == 2, "select test group");
    if (strcmp(argv[1], "reference") == 0) Test_Reference();
    else if (strcmp(argv[1], "stream") == 0) Test_Stream();
    else if (strcmp(argv[1], "polynomial") == 0) Test_Polynomial();
    else if (strcmp(argv[1], "lifecycle") == 0) Test_Lifecycle();
    else if (strcmp(argv[1], "boundary") == 0) Test_Boundary();
    else Check(false, "unknown group");
    printf("PASS %s: %llu checks; object=%u bytes\n", argv[1], checks,
           (unsigned)sizeof(Class_Filter_Polynomial));
    return 0;
}

#include "alg_trajectory.h"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>

static unsigned long long checks = 0;
static unsigned random_state = 73921;
static int scenario = -1;

static void Check(bool condition, const char *message)
{
    checks++;
    if (!condition)
    {
        std::fprintf(stderr, "FAIL scenario=%d: %s\n", scenario, message);
        std::exit(1);
    }
}

static double Random(double lower, double upper)
{
    random_state ^= random_state << 13;
    random_state ^= random_state >> 17;
    random_state ^= random_state << 5;
    return lower + (upper - lower) * (random_state / 4294967296.0);
}

class Test_Trajectory : public Class_Trajectory
{
public:
    double P() const { return State.Position; }
    double V() const { return State.Velocity; }
    double A() const { return State.Acceleration; }
    double Dt() const { return D_T; }
    double Stop_Position() const
    {
        Struct_Profile profile = {};
        return State.Position + Build_Position(0.0, 0.0, &profile);
    }

    double Duration() const
    {
        double duration = 0.0;
        for (uint8_t i = 0; i < Profile.Length; i++) duration += Profile.Segment[i].Duration;
        return duration;
    }

    // Independently integrate the generated jerk schedule in long double and
    // inspect all acceleration endpoints and interior velocity extrema.
    void Audit(double p0, double v0, double a0, bool position_goal, double target) const
    {
        long double p = 0.0L, v = v0, a = a0, magnitude = 0.0L;
        for (uint8_t i = 0; i < Profile.Length; i++)
        {
            if (Profile.Length == 7 && i == 3)
            {
                Check(fabsl(a) <= Acceleration_Max*1e-12L, "zero acceleration at cruise boundary");
                a = 0.0L;
            }
            long double t = Profile.Segment[i].Duration;
            long double j = Profile.Segment[i].Jerk;
            Check(t >= 0.0L && isfinite((double)t), "nonnegative finite segment duration");
            Check(fabsl(j) <= Jerk_Max, "jerk bound");
            if (j != 0.0L)
            {
                long double extremum = -a / j;
                if (extremum > 0.0L && extremum < t)
                {
                    long double peak = v + a * extremum + j * extremum * extremum / 2;
                    Check(fabsl(peak) <= Velocity_Max * (1.0L + 1e-9L), "interior velocity bound");
                }
            }
            magnitude += fabsl(v*t) + fabsl(a*t*t/2) + fabsl(j*t*t*t/6);
            p += v*t + a*t*t/2 + j*t*t*t/6;
            v += a*t + j*t*t/2;
            a += j*t;
            Check(fabsl(v) <= Velocity_Max * (1.0L + 1e-9L), "segment-end velocity bound");
            Check(fabsl(a) <= Acceleration_Max * (1.0L + 1e-9L), "segment-end acceleration bound");
        }
        Check(fabsl(v - (position_goal ? 0.0L : target)) <= Velocity_Max*1e-9L, "terminal velocity");
        Check(fabsl(a) <= Acceleration_Max*1e-9L, "terminal acceleration");
        if (position_goal)
        {
            long double tolerance = 1e-8L*(1+fabsl(target-p0)) + 32*DBL_EPSILON*magnitude;
            if (fabsl(p - ((long double)target-p0)) > tolerance)
            {
                std::fprintf(stderr, "endpoint=%.17g target=%.17g start=(%.17g %.17g %.17g) limits=(%.17g %.17g %.17g)\n",
                    (double)p+p0, target, p0, v0, a0, Velocity_Max, Acceleration_Max, Jerk_Max);
            }
            Check(fabsl(p - ((long double)target-p0)) <= tolerance, "terminal position");
        }
    }

    void Step()
    {
        double p = P(), v = V(), a = A();
        Enum_Trajectory_Status result = TIM_Calculate_PeriodElapsedCallback();
        if (result == TRAJECTORY_ERROR)
        {
            std::fprintf(stderr, "state=(%.17g %.17g %.17g) goal=%d %.17g limits=(%.17g %.17g %.17g)\n",
                p, v, a, Target_Type, Target, Velocity_Max, Acceleration_Max, Jerk_Max);
        }
        Check(result != TRAJECTORY_ERROR, "valid state/target must produce a trajectory");
        Check(fabs(A()-a) <= Jerk_Max*D_T + 1e-9*Acceleration_Max, "sample acceleration continuity");
        Check(fabs(V()-v-a*D_T) <= Jerk_Max*D_T*D_T/2 + 1e-9*Velocity_Max, "sample velocity continuity");
        Check(fabs(P()-p-v*D_T-a*D_T*D_T/2) <= Jerk_Max*D_T*D_T*D_T/6 +
            1e-9*(1+fabs(P())), "sample position continuity");
        Check(fabs(V()) <= Velocity_Max*(1+1e-9), "sample velocity bound");
        Check(fabs(A()) <= Acceleration_Max*(1+1e-9), "sample acceleration bound");
    }

    void Finish()
    {
        for (int i = 0; i < 1000000; i++)
        {
            Step();
            if (Get_Status() == TRAJECTORY_FINISHED) return;
        }
        Check(false, "trajectory must finish within test horizon");
    }

    // Inject a numerical planning failure after Reset has installed a valid
    // velocity transition; the old trajectory must remain executable.
    void Fail_Next_Plan()
    {
        Target_Type = TARGET_POSITION;
        Target = std::numeric_limits<double>::quiet_NaN();
        Target_Changed = true;
    }
};

static void Contract()
{
    Test_Trajectory t;
    Check(t.Get_Status() == TRAJECTORY_UNINITIALIZED, "initial status");
    Check(!t.Reset(0) && !t.Set_Target_Position(1) && !t.Set_Target_Velocity(1), "uninitialized inputs");
    Check(t.TIM_Calculate_PeriodElapsedCallback() == TRAJECTORY_UNINITIALIZED, "uninitialized tick");
    Check(!t.Init(0, 1, 1) && !t.Init(1, -1, 1) && !t.Init(1, 1, 0) && !t.Init(1, 1, 1, 0), "invalid config");
    Check(t.Init(2, 4, 8), "init");
    Check(t.Get_Status() == TRAJECTORY_FINISHED, "initialized at rest");
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    Check(!t.Init(nan, 4, 8) && !t.Init(2, inf, 8), "nonfinite config");
    Check(!t.Reset(nan) && !t.Reset(0, inf) && !t.Reset(0, 0, nan), "nonfinite state");
    Check(!t.Reset(0, 2, 1) && !t.Reset(0, -2, -1), "outward acceleration at velocity boundary");
    Check(!t.Reset(0, 0, 5) && !t.Reset(0, 3), "state limits");
    Check(!t.Set_Target_Position(inf) && !t.Set_Target_Velocity(nan) && !t.Set_Target_Velocity(3), "invalid target");

    const float targets[] = {0.0f, 1.0f, -1.0f, 10.0f, -10.0f, 1e-9f, -1e-9f};
    for (float target : targets)
    {
        Check(t.Reset(0), "reset rest");
        Check(t.Set_Target_Position(target), "position target");
        t.Step();
        t.Audit(0, 0, 0, true, target);
        if (fabsf(target) == 1.0f)
            Check(fabs(t.Duration() - 4*cbrt(1.0/16)) < 1e-10, "analytic triangular S duration");
        if (fabsf(target) == 10.0f)
            Check(fabs(t.Duration() - 6) < 1e-10, "analytic cruise S duration");
        t.Finish();
        Check(t.Get_Position() == target && t.Get_Velocity() == 0 && t.Get_Acceleration() == 0, "position hold");
        for (int k = 0; k < 20; k++) t.Step();
        Check(t.Get_Position() == target, "hold after completion");
    }

    Check(t.Init(2, 4, 8, 2.0f), "large dt init");
    Check(t.Set_Target_Position(1), "large dt position");
    t.Step();
    Check(t.Get_Status() == TRAJECTORY_FINISHED && t.Get_Position() == 1, "cross all phases in one tick");
    Check(t.Reset(0) && t.Set_Target_Velocity(1), "large dt velocity");
    t.Step();
    Check(fabs(t.P() - (2.0 - sqrt(0.5)/2)) < 1e-10, "remaining time cruises after velocity arrival");
    double p = t.P();
    t.Step();
    Check(fabs(t.P() - p - 2) < 1e-12 && t.V() == 1, "finished velocity keeps moving");

    Test_Trajectory repeated, once;
    Check(repeated.Init(3, 5, 9) && once.Init(3, 5, 9), "repeated target init");
    Check(repeated.Set_Target_Position(4) && once.Set_Target_Position(4), "repeated target start");
    for (int i = 0; i < 1000; i++)
    {
        repeated.Set_Target_Position(4);
        repeated.Step();
        once.Step();
        Check(repeated.P() == once.P() && repeated.V() == once.V() && repeated.A() == once.A(), "same target does not restart");
    }
    p = repeated.P();
    double v = repeated.V(), a = repeated.A();
    repeated.Set_Target_Position(-10);
    repeated.Set_Target_Velocity(-1);
    repeated.Set_Target_Position(0.3f);
    Check(repeated.P() == p && repeated.V() == v && repeated.A() == a, "setters preserve state");
    repeated.Step();
    repeated.Audit(p, v, a, true, 0.3f);
    repeated.Finish();
    Check(repeated.Get_Position() == 0.3f, "last target wins");

    Check(t.Init(1, 1, 1, 0.01f) && t.Reset(0, 0.9f, 0.2f), "moving feasible reset");
    Test_Trajectory fallback = t;
    fallback.Fail_Next_Plan();
    t.Step();
    Check(fallback.TIM_Calculate_PeriodElapsedCallback() == TRAJECTORY_ERROR, "failed plan reports error");
    Check(fallback.P() == t.P() && fallback.V() == t.V() && fallback.A() == t.A(), "failed plan preserves previous trajectory");
    fallback.Set_Target_Velocity(0.9f);
    fallback.Step();
    Check(fallback.Get_Status() != TRAJECTORY_ERROR, "new valid target clears error");
    t.Finish();
    Check(t.Get_Velocity() == 0.9f && t.Get_Acceleration() == 0, "reset defaults to current velocity");

    // A rounded nonzero initial acceleration must not leave a residual that
    // accumulates over a long cruise. Exercise both planning and sampling.
    Check(t.Init(1, 1, 1, 1000) && t.Reset(0, -1, 0.8f), "long cruise initial state");
    t.Set_Target_Position(-1000000);
    t.Step();
    t.Audit(0, -1, 0.8f, true, -1000000);
    Check(t.A() == 0, "cruise acceleration is exactly zero");
    t.Finish();
    Check(t.Get_Position() == -1000000 && t.V() == 0 && t.A() == 0, "long cruise endpoint");

    for (int sign = -1; sign <= 1; sign += 2)
    {
        float target = sign*0.3535534143447876f;
        Check(t.Init(1, 1, 1) && t.Reset(0, sign*0.5f), "near-stop initial state");
        t.Set_Target_Position(target);
        t.Step();
        t.Audit(0, sign*0.5f, 0, true, target);
        t.Finish();
        Check(t.Get_Position() == target && t.V() == 0 && t.A() == 0, "near-stop sampled endpoint");
    }
}

static void Random_Profiles(double exponent = 2)
{
    for (scenario = 0; scenario < 30000; scenario++)
    {
        float vm = (float)pow(10.0, Random(-exponent, exponent));
        float am = (float)pow(10.0, Random(-exponent, exponent));
        float jm = (float)pow(10.0, Random(-exponent, exponent));
        float v = (float)Random(-0.999*vm, 0.999*vm);
        double sign = Random(0, 1) < 0.5 ? -1.0 : 1.0;
        float a = (float)(sign*Random(0, 0.999)*fmin(am, sqrt(2*jm*(vm-sign*v))));
        float p = (float)Random(-10, 10);
        float target = (float)(p + Random(-3, 3)*(vm*vm/am + vm*sqrt(vm/jm)));
        Test_Trajectory t;
        Check(t.Init(vm, am, jm), "random init");
        Check(t.Reset(p, v, a), "random feasible reset");
        Check(t.Set_Target_Position(target), "random target");
        t.Step();
        t.Audit(p, v, a, true, target);
        if (scenario % 50 == 0 && t.Duration() < 10)
        {
            t.Finish();
            Check(t.Get_Position() == target, "random sampled endpoint");
        }
        t.Reset(p, v, a);
        target = (float)Random(-vm, vm);
        t.Set_Target_Velocity(target);
        t.Step();
        t.Audit(p, v, a, false, target);
    }
}

static void Boundary()
{
    const float configs[][3] = {
        {1, 1, 1}, {2, 4, 12}, {100, 0.01f, 100},
        {0.01f, 100, 0.01f}, {1000, 0.001f, 1000},
    };
    scenario = 0;
    for (unsigned c = 0; c < sizeof(configs)/sizeof(configs[0]); c++)
    {
        float vm = configs[c][0], am = configs[c][1], jm = configs[c][2];
        for (int vi = -10; vi <= 10; vi++)
        {
            float v = vm*vi/10;
            for (int ai = -10; ai <= 10; ai++)
            {
                float a = am*ai/10;
                double peak = v + (double)a*fabs(a)/(2*jm);
                if (fabs(peak) > vm || fabs(v) > vm || fabs(a) > am) continue;
                scenario++;
                Test_Trajectory t;
                Check(t.Init(vm, am, jm) && t.Reset(0, v, a), "boundary initial state");
                float stop = (float)t.Stop_Position();
                const float targets[] = {
                    stop, nextafterf(stop, INFINITY), nextafterf(stop, -INFINITY),
                    0, 1000, -1000, 1e6f, -1e6f,
                };
                for (float target : targets)
                {
                    if (Basic_Math_Is_Invalid_Float(target)) continue;
                    Check(t.Reset(0, v, a) && t.Set_Target_Position(target), "boundary position target");
                    t.Step();
                    t.Audit(0, v, a, true, target);
                }
                float natural = (float)peak;
                const float speeds[] = {
                    v, natural, nextafterf(natural, INFINITY), nextafterf(natural, -INFINITY),
                    0, vm, -vm,
                };
                for (float target : speeds)
                {
                    if (fabs(target) > vm || Basic_Math_Is_Invalid_Float(target)) continue;
                    Check(t.Reset(0, v, a) && t.Set_Target_Velocity(target), "boundary velocity target");
                    t.Step();
                    t.Audit(0, v, a, false, target);
                }
            }
        }
    }
}

static void Retarget()
{
    Test_Trajectory t;
    Check(t.Init(3, 7, 23), "retarget init");
    for (scenario = 0; scenario < 100000; scenario++)
    {
        bool position_goal = scenario % 4 != 0;
        float target = position_goal ? (float)Random(-10, 10) : (float)Random(-3, 3);
        double p = t.P(), v = t.V(), a = t.A();
        if (position_goal) Check(t.Set_Target_Position(target), "online position goal");
        else Check(t.Set_Target_Velocity(target), "online velocity goal");
        t.Step();
        t.Audit(p, v, a, position_goal, target);
    }
    t.Set_Target_Position(0);
    t.Finish();
    Check(t.Get_Position() == 0 && t.Get_Velocity() == 0 && t.Get_Acceleration() == 0, "settles after retarget stream");

    t.Set_Target_Velocity(3);
    t.Finish();
    double p = t.P();
    t.Set_Target_Position((float)(p + 0.01));
    bool crossed = false, reversed = false;
    for (int i = 0; i < 10000; i++)
    {
        t.Step();
        crossed = crossed || t.P() > p + 0.01;
        reversed = reversed || t.V() < 0;
        if (t.Get_Status() == TRAJECTORY_FINISHED) break;
    }
    Check(crossed && reversed && t.Get_Status() == TRAJECTORY_FINISHED, "nearby target allows necessary overshoot and return");

    t.Reset(0);
    for (scenario = 0; scenario < 10000; scenario++)
    {
        double time = scenario * t.Dt();
        float target = (float)(sin(time*2.0) + 0.05*sin(time*31.0));
        double p0 = t.P(), v0 = t.V(), a0 = t.A();
        t.Set_Target_Position(target);
        t.Step();
        t.Audit(p0, v0, a0, true, target);
    }
    t.Set_Target_Velocity(0);
    t.Finish();
    Check(t.Get_Velocity() == 0 && t.Get_Acceleration() == 0, "smooth stop after moving signal");
}

static void Trace(const char *path)
{
    FILE *file = std::fopen(path, "w");
    Check(file != nullptr, "trace file");
    std::fprintf(file, "time,mode,target,position,velocity,acceleration,average_jerk\n");
    Test_Trajectory t;
    t.Init(2, 4, 12, 0.001f);
    float target = 2;
    int mode = 0;
    t.Set_Target_Position(target);
    for (scenario = 0; scenario < 7000; scenario++)
    {
        if (scenario == 650) { target = -0.5f; t.Set_Target_Position(target); }
        if (scenario == 2200) { target = 1; t.Set_Target_Position(target); }
        if (scenario == 4000) { mode = 1; target = -0.8f; t.Set_Target_Velocity(target); }
        if (scenario == 5300) { target = 0; t.Set_Target_Velocity(target); }
        double previous = t.A();
        t.Step();
        std::fprintf(file, "%.9g,%d,%.9g,%.12g,%.12g,%.12g,%.12g\n",
            (scenario+1)*t.Dt(), mode, target, t.P(), t.V(), t.A(), (t.A()-previous)/t.Dt());
    }
    std::fclose(file);
}

int main(int argc, char **argv)
{
    if (argc != 2 && argc != 3) return 2;
    auto start = std::chrono::steady_clock::now();
    if (std::strcmp(argv[1], "contract") == 0) Contract();
    else if (std::strcmp(argv[1], "random") == 0) Random_Profiles();
    else if (std::strcmp(argv[1], "scale") == 0) Random_Profiles(3);
    else if (std::strcmp(argv[1], "boundary") == 0) Boundary();
    else if (std::strcmp(argv[1], "retarget") == 0) Retarget();
    else if (std::strcmp(argv[1], "trace") == 0 && argc == 3) Trace(argv[2]);
    else return 2;
    double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
    std::printf("PASS %s: %llu checks, %.3f host seconds, object=%zu bytes\n", argv[1], checks, seconds, sizeof(Test_Trajectory));
    return 0;
}

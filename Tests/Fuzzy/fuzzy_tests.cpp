#include "alg_fuzzy.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>

static void Check(bool condition, const char *message)
{
    if (!condition)
    {
        fprintf(stderr, "FAIL: %s\n", message);
        exit(1);
    }
}

// 参考模型逐个计算所有隶属度和规则, 不使用生产代码的区间搜索或插值。
static double Membership(double x, const float *nodes, int count, int index)
{
    if (index == 0 && x <= nodes[0]) return 1.0;
    if (index == count - 1 && x >= nodes[count - 1]) return 1.0;
    if (x == nodes[index]) return 1.0;
    if (index > 0 && x > nodes[index - 1] && x < nodes[index])
        return (x - nodes[index - 1]) / ((double)nodes[index] - nodes[index - 1]);
    if (index + 1 < count && x > nodes[index] && x < nodes[index + 1])
        return (nodes[index + 1] - x) / ((double)nodes[index + 1] - nodes[index]);
    return 0.0;
}

static void Reference(const Struct_Fuzzy_Sugeno_Config &config, float x, float y, double *out)
{
    double sum = 0.0;
    for (int o = 0; o < config.Output_Count; o++) out[o] = 0.0;
    for (int i = 0; i < config.Input_1_Count; i++)
    {
        for (int j = 0; j < config.Input_2_Count; j++)
        {
            double weight = Membership(x, config.Input_1_Nodes, config.Input_1_Count, i) *
                            Membership(y, config.Input_2_Nodes, config.Input_2_Count, j);
            sum += weight;
            for (int o = 0; o < config.Output_Count; o++)
                out[o] += weight * config.Rule_Table[(i * config.Input_2_Count + j) * config.Output_Count + o];
        }
    }
    Check(fabs(sum - 1.0) < 1e-12, "reference membership partition");
    for (int o = 0; o < config.Output_Count; o++) out[o] /= sum;
}

static uint32_t Seed = 20260918;
static float Random()
{
    Seed = Seed * 1664525 + 1013904223;
    return (float)(Seed >> 8) / 16777216.0f;
}

int main()
{
    const float x_nodes[] = {-3.0f, -0.7f, -0.3f, 0.0f, 0.2f, 0.65f, 2.0f};
    const float y_nodes[] = {-5.0f, -1.0f, 0.0f, 0.4f, 6.0f};
    const int output_counts[] = {1, 3, 5};
    double max_error = 0.0;
    int samples = 0;
    for (int outputs : output_counts)
    {
        float table[7 * 5 * 5];
        for (int i = 0; i < 7 * 5 * outputs; i++) table[i] = 6.0f * Random() - 3.0f;
        Struct_Fuzzy_Sugeno_Config config;
        config.Input_1_Nodes = x_nodes;
        config.Input_2_Nodes = y_nodes;
        config.Input_1_Count = 7;
        config.Input_2_Count = 5;
        config.Rule_Table = table;
        config.Output_Count = outputs;
        config.Rule_Table_Length = 7 * 5 * outputs;
        Class_Fuzzy_Sugeno fuzzy;
        float actual[7];
        double expected[5];
        for (float &v : actual) v = 12345.0f;
        Check(!fuzzy.Calculate(0.0f, 0.0f, actual + 1), "uninitialized rejected");
        Check(actual[1] == 12345.0f, "uninitialized preserves output");
        Check(fuzzy.Init(config), "valid nonuniform configuration");
        Check(fuzzy.Get_Output_Count() == outputs, "output count");
        for (int i = 0; i < 30000; i++)
        {
            float x = 10.0f * Random() - 5.0f;
            float y = 16.0f * Random() - 8.0f;
            Check(fuzzy.Calculate(x, y, actual + 1), "calculate");
            Reference(config, x, y, expected);
            for (int o = 0; o < outputs; o++)
            {
                double error = fabs(actual[o + 1] - expected[o]);
                if (error > max_error) max_error = error;
                Check(error < 2e-6, "full Sugeno reference agreement");
                Check(actual[o + 1] >= -3.000001f && actual[o + 1] <= 3.000001f, "convex output bounds");
            }
            Check(actual[0] == 12345.0f && actual[outputs + 1] == 12345.0f, "output buffer bounds");
            samples++;
        }
        for (int i = 0; i < 7; i++)
            for (int j = 0; j < 5; j++)
            {
                Check(fuzzy.Calculate(x_nodes[i], y_nodes[j], actual + 1), "exact node");
                for (int o = 0; o < outputs; o++)
                    Check(actual[o + 1] == table[(i * 5 + j) * outputs + o], "node returns rule exactly");
            }
        for (int i = 1; i < 6; i++)
        {
            float left[5], right[5];
            fuzzy.Calculate(x_nodes[i] - 0.000001f, 0.13f, left);
            fuzzy.Calculate(x_nodes[i] + 0.000001f, 0.13f, right);
            for (int o = 0; o < outputs; o++) Check(fabs(left[o] - right[o]) < 0.0001, "continuous at nodes");
        }
        const float invalid[] = {NAN, INFINITY, -INFINITY, FLT_MIN / 2.0f};
        for (float value : invalid)
        {
            actual[1] = 12345.0f;
            Check(!fuzzy.Calculate(value, 0.0f, actual + 1), "reject invalid first input");
            Check(!fuzzy.Calculate(0.0f, value, actual + 1), "reject invalid second input");
            Check(actual[1] == 12345.0f, "invalid input preserves output");
        }
        Check(!fuzzy.Calculate(0.0f, 0.0f, NULL), "null output rejected");

        Struct_Fuzzy_Sugeno_Config bad = config;
        bad.Rule_Table_Length--;
        Check(!fuzzy.Init(bad), "incomplete rule table");
        bad = config; bad.Rule_Table = NULL;
        Check(!fuzzy.Init(bad), "null rules");
        bad = config; bad.Output_Count = 0;
        Check(!fuzzy.Init(bad), "zero outputs");
        bad = config; bad.Input_1_Count = 1;
        Check(!fuzzy.Init(bad), "too few nodes");
        bad = config; bad.Input_2_Nodes = NULL;
        Check(!fuzzy.Init(bad), "null nodes");
        float bad_nodes[] = {0.0f, 0.0f};
        bad = config; bad.Input_1_Count = 2; bad.Input_1_Nodes = bad_nodes;
        Check(!fuzzy.Init(bad), "duplicate nodes");
        bad_nodes[1] = -1.0f;
        Check(!fuzzy.Init(bad), "descending nodes");
        bad_nodes[1] = NAN;
        Check(!fuzzy.Init(bad), "invalid nodes");
        bad_nodes[0] = -FLT_MAX; bad_nodes[1] = FLT_MAX;
        Check(!fuzzy.Init(bad), "unrepresentable interval width");
        float saved_rule = table[0]; table[0] = NAN;
        Check(!fuzzy.Init(config), "invalid rule");
        table[0] = saved_rule;
        fuzzy.Calculate(0.17f, -0.35f, actual + 1);
        Reference(config, 0.17f, -0.35f, expected);
        for (int o = 0; o < outputs; o++) Check(fabs(actual[o + 1] - expected[o]) < 2e-6, "failed init preserves configuration");
    }

    // 最小2x2规则表: y = x1 + 2*x2, 输入越界保持端点。
    const float nodes[] = {0.0f, 1.0f};
    const float rules[] = {0.0f, 2.0f, 1.0f, 3.0f};
    Struct_Fuzzy_Sugeno_Config config;
    config.Input_1_Nodes = nodes; config.Input_2_Nodes = nodes;
    config.Input_1_Count = 2; config.Input_2_Count = 2;
    config.Rule_Table = rules; config.Output_Count = 1; config.Rule_Table_Length = 4;
    Class_Fuzzy_Sugeno fuzzy;
    Check(fuzzy.Init(config), "minimum configuration");
    float result;
    fuzzy.Calculate(0.25f, 0.5f, &result);
    Check(result == 1.25f, "analytic affine surface");
    fuzzy.Calculate(-100.0f, 100.0f, &result);
    Check(result == 2.0f, "both shoulder boundaries");
    printf("PASS: full_rule_reference_samples=%d max_abs_error=%.9g\n", samples, max_error);
    printf("PASS: 1/3/5 outputs, nodes, shoulders, continuity, buffer bounds, invalid configuration/input\n");
    return 0;
}

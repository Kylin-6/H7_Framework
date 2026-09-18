# 零阶 Sugeno 推理验证

直接编译生产 `alg_fuzzy.cpp` 和 `alg_basic.cpp`，仅替换 ARM 数学头文件。
参考模型逐项计算全部隶属度、规则权重和加权平均，与生产代码的局部插值独立。

```powershell
cmake -S Tests/Fuzzy -B build/Fuzzy -G Ninja -DCMAKE_CXX_COMPILER=g++ -DCMAKE_BUILD_TYPE=Debug
cmake --build build/Fuzzy
ctest --test-dir build/Fuzzy --output-on-failure -V
```

覆盖非均匀节点、1/3/5个输出、九万个随机输入、精确节点、边界肩形、连续性、输出缓冲区边界，以及配置和输入的失败行为。
只验证推理计算，不证明具体规则表的闭环稳定性或控制效果。

## 最小调用示例

以下规则仅演示线性曲面 `out = input1 + 2 * input2`，不代表任何设备的控制参数。

```c
static const float nodes[] = {0.0f, 1.0f};
// 按[input1节点][input2节点][输出]展开, 多输出时每条规则的各输出连续存放。
static const float rules[] = {0.0f, 2.0f, 1.0f, 3.0f};
Class_Fuzzy_Sugeno fuzzy;
Struct_Fuzzy_Sugeno_Config config;
config.Input_1_Nodes = nodes;
config.Input_2_Nodes = nodes;
config.Input_1_Count = 2;
config.Input_2_Count = 2;
config.Rule_Table = rules;
config.Output_Count = 1;
config.Rule_Table_Length = 4;
if (fuzzy.Init(config))
{
    float out;
    fuzzy.Calculate(0.25f, 0.5f, &out); // out = 1.25
}
```

节点及规则数组必须在使用期间保持有效且只读。输出缓冲区至少容纳
`Output_Count`个float，不得覆盖节点或规则表。运行时输入采用节点的单位；
超范围保持边界规则，输入缩放、微分滤波、PID增益映射及执行器限幅由应用负责。

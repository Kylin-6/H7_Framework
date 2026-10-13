# RLS 递推最小二乘

从 H7_BSP `4c1dbd4` 移植 `Class_RLS`，用于线性参数模型 `y = φᵀθ` 的在线辨识。
固定容量 6 维，不动态分配；参数、协方差与中间计算使用 double，输入及参数输出为 float。

| 接口 | 用法 |
| --- | --- |
| `Init(dimension, forgetting_factor, initial_covariance, initial_parameters)` | 维度 1～6，遗忘因子 `(0,1]`，初始协方差为正；可省略初始参数，默认全零 |
| `Update(features, measurement)` | `features` 至少包含 dimension 个元素；每个新观测调用一次，返回是否更新成功 |
| `Get_Parameters(out, capacity)` | capacity 至少为 dimension；未初始化或缓冲区无效时返回 false |
| `Get_Parameter(index)` | 读取单项；未初始化或越界时返回零 |
| `Get_Dimension()` / `Get_Initialized_Flag()` | 查询当前维度和初始化状态 |
| `Reset()` | 恢复最近一次成功 Init 的初始参数及对角协方差 |

`Init()` 失败保留原状态；`Update()` 拒绝非有限输入、非法分母、溢出及非正定候选协方差，
失败不提交参数和协方差。遗忘因子 1 表示不遗忘，小于 1 时更快跟随变化，也更依赖持续有效激励。
模型单位由调用方定义，特征缩放与测量时序必须一致。

```cpp
Class_RLS estimator;
const float initial[2] = {0.0f, 0.0f};
if (estimator.Init(2, 0.99f, 100.0f, initial))
{
    const float features[2] = {1.0f, 2.0f};
    if (estimator.Update(features, 3.0f))
    {
        float parameters[2];
        estimator.Get_Parameters(parameters, 2);
    }
}
```

由同一任务持有实例，跨上下文访问需外部同步。只对新观测更新，不以重复旧样本补齐周期。
功率模型的约束、系数边界及驱动工况筛选由
[PowerControl](../../../Application/PowerControl/README.md) 负责。

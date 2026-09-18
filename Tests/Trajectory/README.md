# 单轴三阶轨迹生成器

实现位于 `User_File/Middleware/Algorithm/Trajectory/alg_trajectory.h/.cpp`，类名 `Class_Trajectory`。
一个对象负责一个轴，位置、速度、加速度连续，速度、加速度和 jerk 使用正负对称限幅。
纯算法，不创建任务、不分配堆内存、不读取硬件反馈；对象由一个控制上下文使用。

## 使用契约

- `Init(v_max, a_max, j_max, dt)`：参数须有限且大于零；成功后在零位置静止。配置失败保留旧状态。
- `Reset(p, v, a)`：对齐初始状态，默认 `v=a=0`。失败保留旧状态；成功后默认保持给定速度，并将非零加速度平滑收至零。需要其他目标时紧接着设置目标。
- `Set_Target_Position(p)`：到达指定位置并以零速度、零加速度结束。
- `Set_Target_Velocity(v)`：达到指定速度及零加速度，随后继续匀速运动。速度目标必须在限幅内；目标零速度用于平滑停止。
- 设置目标只保存最新请求；下一周期从当前规划的 `p/v/a` 接续。重复的同类型同数值目标不重规划，周期内多次设置以最后一个为准。
- `TIM_Calculate_PeriodElapsedCallback()`：严格按 `dt` 调用，返回当前状态。
- `Get_Position/Velocity/Acceleration()`：读取本周期参考值；`Get_Status()` 读取状态。
- `TRAJECTORY_FINISHED` 表示达到本次目标状态；速度目标非零时不表示停止。
- `TRAJECTORY_ERROR` 表示候选轨迹未通过数值或约束检查；保留并继续上一条有效轨迹，错误状态保持至新目标或成功 Reset。调用方应检查返回值，不把错误当成新目标已经执行。

Reset 检查初始速度和加速度的幅值，以及将加速度以最大反向 jerk 收至零之前的速度极值。
例如已达到正速度上限时仍有正加速度，不能作为满足全程速度约束的初始状态。
配置在运动期间保持不变；重新 Init 是明确的状态重置。

位置目标不是机械位置边界。新目标落在当前制动距离以内时，允许必要的越过、制动及返回。
目标持续变化时生成受约束的跟随轨迹；目标稳定后收敛到相应终态。
本实现不保证时间最优，不做多轴同步，不约束 snap。

接口使用 float，段时间和内部状态使用 double。位置单位由调用方确定，速度、加速度、jerk 分别使用该单位除以秒、秒平方、秒立方。
浮点数仍有精度和范围限制；不应把数值求解失败通过大幅终态截断隐藏。
输入校验复用 Basic 的现有规则，拒绝 NaN、无穷大和次正规数；状态与目标允许零，约束和周期须大于零。

## 接入示例

```c
Class_Trajectory trajectory;

// 初始化/使能入口，角度示例使用 rad。
bool ready = trajectory.Init(2.0f, 4.0f, 12.0f, 0.001f);
if (ready)
{
    ready = trajectory.Reset(measured_position);
}
if (ready)
{
    ready = trajectory.Set_Target_Position(target_position);
}

// 1 ms 控制入口：检查状态后，将规划位置交给位置环。
Enum_Trajectory_Status status = trajectory.TIM_Calculate_PeriodElapsedCallback();
if (status == TRAJECTORY_RUNNING || status == TRAJECTORY_FINISHED)
{
    position_pid.Set_Target(trajectory.Get_Position());
}

// 运行中可以更新位置目标，或切换到速度目标。
trajectory.Set_Target_Position(new_target_position);
trajectory.Set_Target_Velocity(0.0f);
```

示例最后两行是两种独立操作；若连续执行，则最后的零速度目标生效。
周期内不要反复 Reset，也不要用实际反馈直接覆盖规划状态。跟踪误差由后续闭环处理。

## 求解方法

速度过渡从任意可行的初始 `v/a` 到目标速度及零加速度，采用最多三个恒 jerk 片段。
先根据收加速度所产生的自然速度增量选择 jerk 方向，再解加速度峰值；达到加速度上限时补恒加速度段。

位置轨迹由以下片段组成：

```text
当前 v/a → 中间速度且 a=0 → 可选匀速段 → v=0 且 a=0
   最多 3 段                 1 段            最多 3 段
```

将不含匀速段的总位移记为 `D(vc)`。它关于中间速度连续，但不保证全局单调。
若目标超出某个速度端点对应的位移，可在该方向的速度上限补匀速距离；否则利用异号括区二分求根。
自然停止位置用于缩小括区，最多进行 64 次二分；不假设 `D(vc)` 全局单调，也不依赖无界迭代。
中间速度达到浮点分辨率时，选择同方向位移不足的括区端点，增加极短匀速段补足剩余距离，避免在自然刹停点附近因速度量化而求解失败。

这是对当前轨迹的直接接续，不将每次更新拆成“先停住再出发”。选择的是可行轨迹，可能比时间最优轨迹更慢。
两个求解入口共用恒 jerk 多项式执行，跨段时拆分周期，速度到达后的剩余周期继续按目标速度前进。
候选轨迹检查每段端点、段内加速度过零处的速度极值及终态；仅在检查通过后替换当前轨迹。
终态位置残差阈值为 `1e-9*(1+abs(本次位移))`，速度和加速度残差阈值为相应上限的 `1e-10`。
终点只消除已验证的浮点残差，不使用输出限幅补救错误轨迹。
第一段速度过渡结束时也检查并清除零加速度的舍入残差，避免它在长匀速段中累积；规划、检查和执行采用同一个边界约定。

## 验证

独立主机工程直接编译生产 Trajectory 和 Basic 源码，复用 Fuzzy 测试的 ARM 数学头替身。
测试包含解析可知的静止起步轨迹、长双精度分段积分和连续时间极值检查。

```powershell
cmake -S Tests/Trajectory -B build/Tests_Trajectory -G Ninja -DCMAKE_CXX_COMPILER=C:/BSP/mingw64/bin/g++.exe -DCMAKE_BUILD_TYPE=Release
cmake --build build/Tests_Trajectory
ctest --test-dir build/Tests_Trajectory --output-on-failure

# 导出位置改目标、反向、速度切换和停止的参考数据。
./build/Tests_Trajectory/trajectory_tests.exe trace trajectory_trace.csv
```

- `contract`：输入契约、失效输入不改变状态、解析时长、微小位移、跨全部片段的大周期、长匀速段零加速度残差、匀速延续、重复目标、最后目标生效、求解失败保留原轨迹。
- `random`：3 万组随机初态，同时检查位置与速度目标，约束独立取 `10^-2` 至 `10^2`。
- `scale`：另 3 万组，约束独立扩展至 `10^-3` 至 `10^3`，检查尺度不同时的数值行为。
- `boundary`：1657 组可行初态，覆盖速度/加速度边界、自然刹停点相邻浮点值、长匀速段和自然速度增量的方向切换。
- `retarget`：10 万次逐周期随机改目标、1 万次变化信号跟随、目标稳定后到达，以及制动距离内目标的越过与返回。

测试根据输出分段表，以 long double 独立积分并检查段内速度极值；不调用生产代码的检查或积分函数作为判据。
大幅越过再返回时，独立终点对照额外计入与积分项量级相关的 double 舍入误差，避免将大数相消误判为算法错误。
周期序列另检查 jerk 对加速度、速度和位置增量所给出的积分界。
生产代码最终取样使用 double；主机扩展精度检查不能替代板上浮点行为与时序验证。

本次 MC02 Debug 编译和链接通过。模块作为算法库注册，尚未接入实际电机控制任务，也未烧录或测量 MC02 上的最坏重规划耗时。
通过 ARM 目标文件的调试类型信息核对，当前编译配置下每个对象占 248 字节；主机 ABI 下为 256 字节。
未使用的算法函数可能被链接器回收；现有固件的 Flash 增量不代表模块启用后的代码占用。

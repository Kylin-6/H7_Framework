# 通用多项式滤波与微分

`Class_Filter_Polynomial` 对固定周期的标量采样做等权滑动最小二乘拟合，在窗口末端输出平滑值及一、二、三阶时间导数。支持 0～3 阶，默认二阶；输入可为位置、温度、电流等标量，单位为 `U` 时导数单位为 `U/s`、`U/s²`、`U/s³`。

## 使用

```c
Class_Filter_Polynomial Filter;

bool initialized = Filter.Init(17, 0.001f, 3);
// initialized为true后, 每个有效等间隔样本调用一次:
Filter.Set_Now(sample);
Filter.TIM_Calculate_PeriodElapsedCallback();
if (Filter.Get_Ready_Flag())
{
    float value = Filter.Get_Out();
    float first = Filter.Get_First_Derivative();
    float second = Filter.Get_Second_Derivative();
    float third = Filter.Get_Third_Derivative();
}
```

- `Init(Window_Size, D_T, Polynomial_Order)` 要求 `0 <= Polynomial_Order <= 3`、`Polynomial_Order < Window_Size <= 33`，采样周期为正常有限正数。有效系数超出正常 `float` 表示范围或整行下溢时返回 `false`，并保留原配置、历史、待处理输入及输出。
- 0 阶是移动平均；0 阶单点是直通。点数等于阶数加一时是插值，原量没有平滑效果。支持奇数和偶数窗口。
- `Get_Polynomial_Order()` 返回配置的拟合阶数。高于拟合阶数的导数恒为零，表示拟合模型的导数，不代表真实信号没有变化。
- 每个样本先 `Set_Now()` 再计算一次；Setter 不推进历史。输入与重置值应有限，调用方保证差值、乘加及结果在 `float` 表示范围内。
- 收满窗口前原量直通、导数为零、`Ready=false`。`Reset(value)` 保留配置、清空计数，原量置为 `value`、导数清零，不用复制样本补满历史。
- `Ready` 仅表示窗口已满。缺测、时间断续、坐标跳变后由调用方重置；较快的控制循环不能重复输入较慢传感器的旧测量来填充窗口。角度跨圈展开、单位转换和数据新鲜度由调用方处理。
- 输出在最新采样位置求值，仍有滤波响应延迟。初始化、重置和更新由同一上下文串行执行。

## H7 实现

初始化用 `double` 直接生成离散正交多项式系数，运行使用 `float`。四组系数和一份环形缓冲内嵌于对象，最多 33 点，无动态分配、RTOS 或 HAL 依赖。环形缓冲分两段遍历，系数按从最老到最新排序，运算先减去当前输入以消除恒定偏置。

每次完整窗口更新执行 `(Polynomial_Order + 1) * Window_Size` 次核心乘加，低阶不计算无效导数；另有样本差值、索引和循环开销。资源占用与时间表现以目标构建及板测为准。

## 主机测试

```powershell
cmake -S Tests/FilterPolynomial -B build/Tests_FilterPolynomial -G Ninja -DCMAKE_CXX_COMPILER=g++ -DCMAKE_BUILD_TYPE=Release
cmake --build build/Tests_FilterPolynomial
ctest --test-dir build/Tests_FilterPolynomial --output-on-failure
```

| 分组 | 覆盖内容 |
| --- | --- |
| `reference` | 所有 126 组合法阶数/窗口、4 种采样周期、0～3 阶导数及未使用系数区，与 SciPy 独立系数对照 |
| `stream` | 全部阶数/窗口、3 种周期、确定性随机流与突变、反复环形回绕，比较独立直接卷积 |
| `polynomial` | 常量、一次、二次和三次信号及解析导数；大恒定偏置与零导数 |
| `lifecycle` | 未初始化、默认参数、完整窗口等待、重置、重复 Setter、重新配置、0 阶直通与移动平均 |
| `boundary` | 非法阶数/窗口/周期、系数上溢与下溢、失败后完整状态保留及待处理样本继续计算 |

参考头由 `generate_reference.py` 使用 SciPy `savgol_coeffs(..., pos=window-1, use='dot')` 生成并纳入版本控制，普通 CTest 不依赖 Python。再生成需要 NumPy/SciPy：

```powershell
python Tests/FilterPolynomial/generate_reference.py
```

流式和解析信号的容差按 `float` 舍入误差、窗口长度及参考系数幅值计算，包含输入量化和微分放大，不把小周期下的导数绝对误差当作恒定精度。主机测试直接编译生产 `.cpp` 和 `alg_basic.cpp`，只以既有主机头替换 CMSIS 处理器依赖。

2026-09-20 验证：上述 5 组共 730,551 个检查点通过，参考头重新生成后内容一致；固件 Debug/Release 编译链接通过。主机和 Cortex-M7 ARM 工具链均确认单实例为 696 字节。板上最坏耗时、实际传感器噪声和控制闭环效果尚未验证。

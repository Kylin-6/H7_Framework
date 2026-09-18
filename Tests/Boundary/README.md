# 边界与失败返回回归

在仓库根目录使用主机 C++ 编译器运行，不加载固件 ARM 工具链：

```powershell
cmake -S Tests/Boundary -B build/Boundary -G Ninja -DCMAKE_CXX_COMPILER=g++ -DCMAKE_BUILD_TYPE=Debug
cmake --build build/Boundary
ctest --test-dir build/Boundary --output-on-failure -V
```

编译器也可指定绝对路径。Windows 使用 MinGW；Linux 使用 GCC/Clang。
测试直接编译生产 PID、矩阵、KF、DM、QDrive、EricTool 源码，仅替换硬件接口。

| 分组 | 行为约定及回归范围 |
| --- | --- |
| `pid` | 本次积分累加后限幅；零 Ki 在计算时清空积分；负 Ki 按幅值限幅；零积分限幅表示不限制；保留积分分离、变速积分和总输出限幅。 |
| `kalman` | 每周期先预测，缺测则跳过更新；X/P 始终为当前估计；连续缺测推进状态与协方差，恢复测量后正常更新。覆盖标量、二维状态和正常预测/更新。 |
| `commands` | DM 动作及模式请求、QDrive 命令传播提交结果。注入队列/周期槽失败，检查原报文、发送路径、失败重试、模式待应答/超时、IRQ 状态恢复。DM 置零失败保留位置展开状态，成功才重置；返回 true 不表示设备确认。 |
| `parser` | UART/USB 均读取回调传入的缓冲区，只读 Length 范围。格式沿用 `variable:value#`，字典为字符串指针数组；变量名 1–99 字节，数值支持负号及单个小数点，至少一个数字且结果有限。非法帧输出 index=-1、value=0。 |

解析测试用不可访问页紧贴输入末尾，覆盖零长度、每个截断位置、非 NUL
结尾帧、99/100 字节变量名、多项字典、非法字符、嵌入 NUL、数值溢出、
空字典及失败后无旧值残留。保持原有首帧语义：第一个 `#` 后的字节忽略；
不添加跨回调拼帧或多帧循环解析。

CAN 失败由桩函数注入；本回归不替代板上队列拥塞、实际发送及电机执行验证。
EricTool 的发送侧可变参数接口和矩阵求逆现有代码可能产生主机编译警告，
本组不调用发送侧可变参数接口。

# 通信边界回归

从 main 的 `f40293d`（CAN）、`2990b25`（USB）、`8513892`（OSPI）移植，
测试与生产实现同步适配 RoboMaster_H7。直接编译生产 BSP 和 CDC C 桥接，
仅替换 HAL、USB 类与 RTOS 服务。

在仓库根目录使用主机编译器运行：

```sh
cmake -S Tests/Communication -B build/Tests_Communication -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build/Tests_Communication
ctest --test-dir build/Tests_Communication --output-on-failure --timeout 10
```

| 模块 | 测试组数 | 覆盖 |
| --- | --- | --- |
| CAN | 6 | 正常帧、跨总线隔离、远程帧过滤、Classic DLC 9..15、HAL 最大 64 字节复制、扩展/FD 帧拒绝 |
| USB | 6 | 双缓冲接收、重新枚举、启动早包、自有发送缓冲、忙时拒绝及 PRIMASK 恢复、非法参数 |
| OSPI | 5 | 两路缓冲路由、命令失败阻止 DMA、长度边界、DMA 提交失败、无效句柄与不支持的双向提交 |

CAN 替身按仓库内 HAL 的 DLC 表复制字节，使用栈保护检测越界；USB 替身保留发送
指针以模拟异步发送期间的缓冲所有权。原 `Tests/CAN` 继续验证命令队列、周期发送
和目标分支的发送失败统计，不能只运行这里的接收测试。

Flash 调用方的失败传播在 `Tests/Initialization` 验证。`HAL_OK` 只代表提交成功，
DMA 接受后的硬件异常、Abort、迟到回调和并发访问恢复不在本批完整覆盖范围。
真实 CAN 帧注入、USB 重连/背压及关中断时长、Flash 读写擦除仍需板上验证。

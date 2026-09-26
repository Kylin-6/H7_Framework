# 初始化与 Flash 失败回归

移植 main 的 `92e6314`、`8513892` 相关测试，适配 RoboMaster_H7 的启动分级接口。
直接编译生产 BMI088、W25Q64JV 与 System_Init，不复制被测函数。

在仓库根目录使用主机编译器运行：

```sh
cmake -S Tests/Initialization -B build/Tests_Initialization -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build/Tests_Initialization
ctest --test-dir build/Tests_Initialization --output-on-failure --timeout 10
```

共 7 组测试：

- 加速度计、陀螺仪：正常启动、ID 缺失、复位后失联、零值/非零配置拒绝、最后一次
  重试成功、超过重试次数、丢失回调；验证加速度计软复位地址和 PWM 启动失败中止。
- BMI088 总入口：部分/全部传感器失败时，不启动后续服务和数据采集。
- Flash 初始化：三字节 JEDEC 校验、第四字节残留、旧 ID 不能冒充新响应、重试上限、
  内存映射失败，以及未初始化时拒绝运行操作。
- Flash 提交失败：ERROR/BUSY/TIMEOUT、轮询拒绝、读写擦除失败状态清理及后续步骤中止。
- Flash Quad：正常命令序列及 WIP 接收所需的数据阶段。
- 系统启动：32 种定时器/IMU/Flash/ADC 失败组合；TIM4/TIM5 失败为 FATAL 并提前结束，
  设备失败为 DEGRADED，全部成功为 READY；验证失败位图、FIFO 启动条件和重复初始化状态重置。

传感器替身在延时期间交付接收回调，验证初始化阶段能够处理读回结果；系统测试
通过设备和 HAL 替身注入失败。五次重试是工程策略，不是芯片手册要求。
主机模型不模拟真实 SPI/DMA 时序、Flash 写入耗时或总线电气条件；仍需板上验证。

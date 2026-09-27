# 达妙云台回归

直接编译生产 Gimbal、Class_DMMotor、PID、Daemon 和 MessageCenter。替身只提供
CAN 注册/提交、寄存器中断屏蔽和统一时间源；反馈按实际 MIT 格式送入生产回调，
断言生产驱动编码后的 CAN 帧，不复制控制函数。

```sh
cmake -S Tests/Gimbal -B build/Tests_Gimbal -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build/Tests_Gimbal
ctest --test-dir build/Tests_Gimbal --output-on-failure
```

共 14 组：配置拒绝、初始化失败、部分初始化失败、稳定时间、Yaw 控制、Pitch 转换、
方向翻转、模式切换、自动恢复、使能超时、发送失败、非法数据、驱动接口和默认参数。
包括 ±π 跨界、速度前馈、转矩/机械限幅、100 ms 反馈过期、10 ms INS 过期、恢复重置
PID、旧目标不重放、DISABLED 不自动使能、20 ms 重试，以及 PRIMASK 嵌套恢复。

测试不模拟物理总线、硬件 FIFO 延迟、电机内部控制器或机构动力学；不能据此认定
示例 PID、MIT 增益、协议量程与机械限位已经适配某一实机。

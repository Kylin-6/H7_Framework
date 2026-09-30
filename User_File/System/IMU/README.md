# INS 状态发布与在线监控

`System_IMU_Configure()` 设置整机 VQF 参数并注册静态 INS Daemon，必须在 BMI088 Init 前调用；注册失败返回 false，System_Init 记录 BMI088/INS 链路不可用并禁止启动采集。

`System_IMU_Publish_State()` 只在 BMI088 已初始化且 Yaw/Pitch/Roll、三轴机体系角速度全部 finite 时 Feed 并发布 INS_State。NaN/Inf 不发布，也不刷新在线时间；原始 SPI/DMA chunk 不 Feed。因此 `System_IMU_IsOnline()` 监控整个采集、解算到有效 INS 输出链路。

监控门限为 30 ms，约 1 kHz 输出下用于低频诊断；Gimbal 仍以 `INS_State_Topic.ReadFresh(..., 10000)` 判断 10 ms 实时控制时效。二者不合并。StatusTask 100 Hz 统一 CheckAll，Daemon 不控制云台模式或整车安全策略；实现不使用动态内存。参见 [Daemon 说明](../Daemon/README.md)。

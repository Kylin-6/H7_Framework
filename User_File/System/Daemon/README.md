# 周期数据源在线监控

正常工作时应持续收到反馈、心跳或数据流的模块，优先持有静态 `Daemon`，在显式初始化时注册到 `DaemonManager`，只在协议和数据校验通过后 `Feed()`。使用固定容量 32 的指针注册表，无 malloc/new、虚函数或运行期注销。重复注册同一对象幂等；注册失败必须让模块初始化失败或保持明确不可用。

`Daemon` 只回答数据源是否活跃、离线多久和发生了什么跃迁，不负责停机、清错、重启设备、消息路由或业务降级。Application 通过 Device 的可用性和业务数据新鲜度决定控制策略，不持有 Daemon 指针。

## 时间与跃迁

- 复用系统绝对时间戳，内部使用自然回绕的 uint32_t 毫秒计数。新对象初始 Offline，收到第一份合法数据后才 Online。
- `Feed()` 可从 ISR 调用；读写状态用短 PRIMASK 临界区保护并恢复原中断状态。
- `IsOnline()` 按查询时刻判断 `age < timeout`，不必等待 StatusTask。它不消费跃迁、不执行回调。
- StatusTask 每 10 ms（100 Hz）统一 `CheckAll()`，报告一次 OfflineToOnline 或 OnlineToOffline；稳定状态返回 None。离线回调仅在 Check 观察到 OnlineToOffline 时调用一次，在临界区外、StatusTask 上下文执行。DM 使用设备自己的离线回调登记安全动作，发送由设备服务执行。
- `OfflineDurationMs()` 从实际超时点计算持续时间，在线为 0；初始未收到数据时从系统零时刻计时。`LastTransition()` 用于观察最近记录的跃迁。
- `SetTimeoutMs()` 仅用于初始化、首次 Feed 前配置，拒绝 0；不能在运行期改变反馈门限。

电机的 `IsOnline()` 与反馈快照统一使用 Daemon 的即时查询（`age < timeout_ms`），控制入口不等待 CheckAll；微秒反馈时间戳仍供诊断使用。Topic `ReadFresh()` 继续判断业务数据的新鲜度，Online 不等同于设备 Ready 或机器人功能可用。

## 当前接入

| 数据源 | 何时 Feed | 监控超时 | 实时/业务判断 |
| --- | --- | --- | --- |
| DM | 总线、ID、DLC、节点校验通过的运动反馈；模式应答不 Feed | 100 ms | 保留 requested/actual/fault；设备服务覆盖安全目标并提交失能 |
| DJI | 总线、CAN ID、DLC、encoder 全部合法且反馈解码完成 | Init 的 feedback_timeout_ms，默认 20 ms | Control/Send 立即清零；独立 ServiceAll 覆盖旧输出并清积分 |
| S.BUS | 完整合法 header/footer 的 25 字节帧，含 frame-lost/failsafe 帧 | 100 ms | Healthy 另看失控位；RemoteInput 50 ms freshness、200 ms 回中恢复保持 |
| INS | BMI088 已初始化且六个姿态/角速度值 finite，实际发布 INS_State 前 | 30 ms | Gimbal 10 ms ReadFresh 保持；原始 SPI chunk 不 Feed |
| BoardTransport | 总线/ID/大小正确、按 RX 时间仍及时、Decode 成功，序号过滤之前 | 100 ms | 重复序号维持链路在线但不刷新 Topic，业务 Topic 可独立过期 |
| Referee | CRC8/CRC16 和完整长度校验通过，已知命令长度正确；未知合法命令只证明链路活性 | 500 ms | Online 不保证每个业务字段已更新 |
| VTM | 完整 CRC16 合法遥控帧，或 CRC8/CRC16 合法图传链路帧 | 300 ms 暂定 | 需按实机帧周期验证；Online 不代表已实现所有业务载荷 |

Referee/VTM 的 C 解析器通过模块内的薄 C 接口访问静态 C++ Daemon，保持原有 C 编译和解析边界。它们当前未在 System_Init 自动绑定 UART；调用各自 Init 后才注册和 Feed。

未来正式接入 Vision 时，应在合法视觉帧后 Feed 独立 Daemon；瞄准目标仍按输入时间戳判断 freshness。当前没有正式 Vision 驱动，不新建占位实现。历史 DBUS 当前不在机器人周期控制链绑定，正式接入时须按同一原则迁移。Buzzer、WS2812、Key、Flash、单次命令接口不因存在对象就注册 Daemon。

## 容量核算

按现有静态应用实例，SingleBoard 取三个硬件应用全部开启，并计入可选 Referee/VTM 初始化：

| 板型 | DM | DJI | S.BUS | INS | Transport | Referee | VTM | 合计 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| SingleBoard | 2 | 11 | 1 | 1 | 0 | 1 | 1 | 17 |
| GimbalBoard | 2 | 3 | 1 | 1 | 1 | 0 | 0 | 8 |
| ChassisBoard | 0 | 8 | 0 | 0 | 1 | 1 | 1 | 11 |

SingleBoard 默认关闭三个硬件应用；Referee/VTM 默认未初始化，因此实际注册数通常更少。GimbalBoard 不编入 Referee/VTM，ChassisBoard 不配置 INS/S.BUS。

CAN 接收注册器目前总容量为 16，DM/DJI/Transport 共享这个上限；加上当前其他四个独立软件数据源，扩展电机数量时的成功注册上界仍最多 20。DaemonManager 保持 MAX_DAEMONS=32；未来增加模块时重新核算，并对注册失败作显式处理。

## 安全职责与调度

CAN RX → Device 校验并 Feed → Daemon 判断 Online/Offline → Device 执行基础 fail-safe → App 决定功能策略。

StatusTask 在 CheckAll 后调用已编入的 DJI、DM `ServiceAll()`，不依赖 App 是否继续调用控制入口。设备负责覆盖旧输出、失能和失败补交；Daemon 不包含电机协议或业务控制。App 保留命令失效、INS 过期、功能停机及恢复策略，不重复逐个电机清零。WS2812 只发送已有颜色缓存，本次不新增显示规则或掉线检测。

设备控制入口即时拒绝离线有效目标；独立保护服务的名义周期为 10 ms，实际延迟还取决于任务调度和 CAN 发送。物理断线时停止帧无法保证送达，软件覆盖也不能撤回已经进入硬件 FIFO 的帧，仍需验证电机端通信超时保护。

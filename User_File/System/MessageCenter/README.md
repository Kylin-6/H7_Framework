# Message Center

本目录提供两条用途不同的消息通道。二者并存，不应为了统一接口而互相替代。

## 静态实时 Topic

`Topic<T>` 使用静态内存、类型安全的直接复制和极短临界区。`Read()` 是非消费式
Latest Value 读取：首次发布后，每个读取者都能持续取得最近一次完整快照。它适合
INS、闭环反馈和其他高频状态；当前 INS -> Gimbal 链路必须使用此通道。

优点是内存和执行时间确定、没有堆与队列开销。缺点是 Topic 必须在编译期定义，
也不提供“是否有未读新消息”的每订阅者状态；需要判断更新时应比较 `Sequence()`。
需要同时使用数据、序号和时间戳时，应调用 `ReadWithMeta()` 获取同一发布帧的
`TopicSnapshot<T>`，不要分别调用 `Read()`、`Sequence()` 和 `Timestamp()` 拼接快照。

## 动态低频 Pub/Sub

`dynamic_message_center.h` 使用字符串注册、注册期链表和每订阅者一个长度为1的
FreeRTOS Queue。每个订阅者独立消费最新未读值，适合未来 RobotCmd、UI、调试或
其他低频应用消息。

动态总线必须遵守以下边界：

- `DynamicMessageCenter_Init()` 只能在 `osKernelInitialize()` 之后、
  `osKernelStart()` 之前调用。
- Publisher/Subscriber 只能在调度器启动前注册；运行期链表保持只读。
- 注册内存来自项目现有 FreeRTOS heap_5，系统生命周期内不注销。
- Topic 名最多31字符，消息大小为1到256字节，一个 Topic 只能有一个 Publisher。
- Publish/Read 仅供任务上下文使用，不允许从 ISR 调用，不阻塞等待。
- INS、控制反馈等高频实时状态不得迁入动态 Queue。

动态通道的优点是应用无需在编译期直接共享对象、订阅者具有独立未读状态。代价是
字符串错误只能在运行期发现，并且每个 Topic、Subscriber 和 Queue 都会消耗堆；
发布开销也随订阅者数量线性增长。

当前 Application 层的 `gimbal_cmd/chassis_cmd/shoot_cmd` 及对应低频反馈使用动态
通道；命令类型仍集中定义在 `message_types.h`。同名命令不再额外创建静态 Topic，
避免两个发布入口造成所有权不清。INS 姿态仍只使用静态 `INS_State_Topic`。

## 开源参考

动态注册模型参考了 MIT 许可的 `basic_framework` 与 `Meta-Embedded-NG`。本实现
保留其字符串 Topic、订阅者先注册和单槽 Latest Queue 思路，但改用 FreeRTOS
heap_5、显式失败返回和启动前注册限制。完整版权说明见
`THIRD_PARTY_NOTICES.md`。GPLv3 的 `omni-wheel-chassis-main` 仅用于比较 App/Driver
边界，没有复制其源码。

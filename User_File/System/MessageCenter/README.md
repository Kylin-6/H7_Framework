# Message Center

本目录统一提供静态消息基础设施、业务消息类型和唯一通道实例。`topic.h` 与
`event_queue.h` 只负责怎么传，`message_types.h` 定义传什么，`message_center.h/.cpp`
集中定义工程使用的 Topic 和 EventQueue。

## Topic

`Topic<T>` 是非阻塞 Latest-Value 通道，适合连续状态和连续控制目标。发布会覆盖旧值，
并在同一短 PRIMASK 临界区内更新数据、sequence、微秒时间戳和 valid 标志。
`Topic::Read()` 始终读取当前最新值；`ReadWithMeta()` 返回同一发布帧的一致快照。

`Publisher<T>` 仅转发到构造时绑定的 Topic。每个 `Subscriber<T>` 独立记录已读
sequence，`Read()` 只在绑定 Topic 首次有效或再次发布后返回 true。两者都不注册、
不查找字符串，也不分配内存。

## EventQueue

`EventQueue<T,N>` 是静态固定容量 FIFO，适合不能被 Latest-Value 覆盖的离散事件。
Push/Pop 均不阻塞；满队列拒绝新元素并累计 overflow。存储、索引和计数全部包含在
对象内部，并使用短 PRIMASK 临界区保护。

使用规则：最新状态或连续命令使用 Topic，离散事件使用 EventQueue，明确的设备控制
使用直接调用，在线检测继续由 Daemon 负责。

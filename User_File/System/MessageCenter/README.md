# Message Center

Message Center 是 H7_BSP 内部的静态、类型安全消息基础设施。它只解决“数据如何在
模块之间传递”，不负责设备驱动、在线检测、安全策略、多板路由或日志。

当前架构只有两种消息语义：

```text
最新状态 / 连续控制目标  → Topic<T>
不可被覆盖的离散事件     → EventQueue<T, N>
明确上下级设备控制        → 直接函数调用
设备在线检测              → Daemon
```

## 1. 设计目标

- 编译期类型安全，不使用字符串 Topic 和运行时类型系统。
- 所有存储静态确定，不使用 `malloc`、`new` 或 STL 动态容器。
- 发布和读取均不阻塞，不创建额外任务。
- 高频 INS 链路与 1 kHz 控制链不增加任务切换。
- 用数据语义而不是更新频率选择通道。
- 并发保护只覆盖必要的数据复制和元数据更新。

## 2. 文件结构

| 文件 | 内容 |
| --- | --- |
| `topic.h` | `TopicSnapshot<T>`、`Topic<T>`、`Publisher<T>`、`Subscriber<T>` |
| `event_queue.h` | 固定容量 FIFO `EventQueue<T,N>` |
| `message_types.h` | INS、应用命令、反馈和射击事件类型 |
| `message_center.h/.cpp` | 全工程唯一 Topic 与 EventQueue 实例 |

`topic.h` 和 `event_queue.h` 是通用基础设施；`message_types.h` 与
`message_center.cpp` 描述本工程实际传递的业务数据。

## 3. Topic：Latest-Value 通道

### 3.1 核心语义

`Topic<T>` 只保存最后一次发布的完整快照。新发布覆盖旧值，不保存历史、不排队。
它适合“接收方开始计算时只需要当前目标或当前状态”的数据：

- INS 姿态与角速度。
- 云台、底盘和发射机构的连续控制目标。
- 云台、底盘和发射机构的当前反馈。

类型 `T` 必须满足 `std::is_trivially_copyable<T>`，保证在短临界区中可以直接复制。

### 3.2 元数据

每次 `Publish()` 同时更新：

| 字段 | 含义 |
| --- | --- |
| `data` | 最新完整消息 |
| `sequence` | 32 位发布序号，每次发布递增，允许自然回绕 |
| `timestamp_us` | 调用 `SYS_Timestamp_Get_Microsecond()` 获取的发布时间 |
| `valid` | 是否至少完成过一次发布 |

需要把数据、序号和时间戳关联到同一帧时，调用 `ReadWithMeta()`；不要分别调用
`Read()`、`Sequence()` 和 `Timestamp()` 后假设它们来自同一次发布。

### 3.3 读取接口的区别

```cpp
Topic<GimbalCmd> topic;

GimbalCmd current;
bool valid = topic.Read(current);  // 首次发布后每次都返回当前最新值

TopicSnapshot<GimbalCmd> snapshot = topic.ReadWithMeta();
```

`Topic::Read()` 是非消费式读取：只要 Topic 有效，就会返回当前值。尚未发布时返回
`false`，并保持调用者输出对象不变。

`Subscriber<T>` 则为每个订阅者保存独立的已读 sequence：

```cpp
Topic<GimbalCmd> topic;
Publisher<GimbalCmd> publisher(topic);
Subscriber<GimbalCmd> controller(topic);
Subscriber<GimbalCmd> logger(topic);

publisher.Publish(command);

controller.Read(command_for_control); // true
controller.Read(command_for_control); // false，同一订阅者没有新帧
logger.Read(command_for_log);          // true，订阅者状态互不影响
```

首次有效快照即使 sequence 恰好为零也能被读取，因为 Subscriber 还保存了
`has_read_`，不会用初始 sequence 值误判。

### 3.4 Publisher / Subscriber

`Publisher<T>` 和 `Subscriber<T>` 只持有绑定 Topic 的引用：

- 构造时不注册、不查找、不分配内存。
- Publisher 的 `Publish()` 直接调用 Topic。
- Subscriber 的 `Read()` 通过一致快照判断是否有新数据。
- 它们不是第二套消息系统，生命周期不能超过被绑定的 Topic。

全局通道具有静态存储期，应用端点也使用静态对象绑定，因此运行期不存在端点注册失败。

## 4. EventQueue：离散事件 FIFO

### 4.1 核心语义

`EventQueue<T,N>` 保存最多 `N` 个离散事件，按 FIFO 顺序消费。类型 `T` 必须可平凡
复制，容量 `N` 必须大于零。

```cpp
EventQueue<ShootEvent, 8U> queue;

bool accepted = queue.Push({ShootEventType::ShootOnce});
ShootEvent event;
bool available = queue.Pop(event);
```

公开接口：

| 接口 | 行为 |
| --- | --- |
| `Push(const T&)` | 队列未满时复制到尾部；满时拒绝并返回 `false` |
| `Pop(T&)` | 非空时取出头部；空时返回 `false` |
| `Empty()` | 返回当前是否为空 |
| `Size()` | 返回当前元素数 |
| `OverflowCount()` | 返回累计满队列拒绝次数 |

满队列不会覆盖最旧事件。`overflow_count` 使用 32 位无符号数自然回绕；调用方若关心
丢失事件，必须检查 `Push()` 返回值或监控计数。

### 4.2 为什么射击使用事件

`ShootCmd` 描述持续状态：总开关、摩擦轮开关和速度、拨弹盘 STOP/REVERSE/BURST。
`ShootEvent` 描述一次动作：`ShootOnce` 或 `ShootTriple`。

若连续两次发布内容相同的 `ShootOnce` 到 Latest-Value Topic，接收端只能看到相同的
最后状态，无法证明动作发生了两次。FIFO 会保留两次成功 Push，因此动作语义不丢失。

当前 `Shoot_Event_Queue` 容量为 8：

- RobotCmd 通过 `RobotCmd_PushShootEvent()` 推入事件并返回是否成功。
- Shoot 在 STOP 且总开关 ON 时每个 1 kHz 周期最多消费一个事件。
- 连续事件在已有角度目标上累加一个或三个弹位。
- BURST/REVERSE 优先并取消事件角度保持。
- 总开关 OFF 时有界排空当时已经排队的事件，避免重新使能后延迟射击。

## 5. 当前静态通道

| 通道 | 发布者 | 消费者 | 语义 |
| --- | --- | --- | --- |
| `INS_State_Topic` | `System_IMU_Publish_State` | Gimbal | 最新姿态与角速度 |
| `Gimbal_Command_Topic` | RobotCmd | Gimbal | 最新云台控制目标 |
| `Chassis_Command_Topic` | RobotCmd | Chassis | 最新底盘速度目标 |
| `Shoot_Command_Topic` | RobotCmd | Shoot | 最新发射连续状态 |
| `Gimbal_Feedback_Topic` | Gimbal | RobotCmd | 最新云台反馈 |
| `Chassis_Feedback_Topic` | Chassis | RobotCmd | 最新底盘反馈 |
| `Shoot_Feedback_Topic` | Shoot | RobotCmd | 最新发射反馈 |
| `Shoot_Event_Queue` | RobotCmd | Shoot | 单发/三连发 FIFO，容量 8 |

`message_center.cpp` 是这些实例的唯一实体定义。禁止在其他翻译单元再定义同类型同用途
Topic，否则会形成两个互不通信的消息入口。

## 6. 数据流

### 6.1 INS 到云台

```text
BMI088 EXTI / SPI DMA 回调
        ↓ 设置线程标志
BMI088_Task（High2）
        ↓ FIFO 解算 + VQF
System_IMU_Publish_State
        ↓ Publisher<INS_State>
INS_State_Topic
        ↓ Subscriber<INS_State>
Gimbal_Update（Control_Task，1 kHz）
```

中间没有消息队列、额外任务或阻塞等待。Topic 只在数据复制与元数据更新时关闭中断。

### 6.2 RobotCmd 与 Application

```text
上层输入
  └─ RobotCmd_SetGimbal / SetChassis / SetShoot
          ↓ dirty 标志
     RobotCmd_Update
          ↓ Publish 最新命令
 Gimbal / Chassis / Shoot Subscriber
          ↓ 直接控制所属 Device
     100 Hz 发布 Feedback
          ↓
 RobotCmd Subscriber 汇总状态
```

命令只在对应 dirty 标志置位时发布。Application 没有读到新命令时继续执行保存的上一帧
目标；反馈控制逻辑保持 1 kHz，Topic 发布频率降到 100 Hz。

## 7. 并发模型

Topic 和 EventQueue 使用 Cortex-M PRIMASK：

```text
读取当前 PRIMASK → 禁止中断 → 复制数据/更新索引与元数据 → DMB → 恢复 PRIMASK
```

恢复原 PRIMASK 而不是直接开中断，因此在本来已经屏蔽中断的上下文中不会错误地提前
使能中断。临界区不包含时间戳获取、业务计算、电机控制或 RTOS 调用。

当前可能访问消息中心的上下文包括：

- BMI088 高优先级任务发布 INS。
- Control_Task 发布命令、读取命令与反馈、处理 ShootEvent。
- 后续 ISR/回调可以使用基础设施，但消息必须足够小，且调用路径不得阻塞。

PRIMASK 会短暂屏蔽所有可屏蔽中断，消息体积必须保持小而可预测。大型数组、图像、日志
或协议帧不应直接放入 Topic/EventQueue；应使用拥有明确生命周期的缓冲或传输模块。

## 8. 新增消息的决策流程

1. 接收方是否只关心开始计算时的最新值？是 → `Topic<T>`。
2. 每一次成功提交是否都必须被观察？是 → `EventQueue<T,N>`。
3. 是否为一个 Application 对其明确拥有 Device 的命令？是 → 直接函数调用。
4. 是否只是判断设备在线/离线？是 → `Daemon::Feed()` / `DaemonManager::CheckAll()`。

不要使用“高频/低频”作为 Topic 与 EventQueue 的判断依据。

## 9. 新增 Topic

```cpp
// 1. message_types.h：定义小型、可平凡复制的数据
struct ExampleState
{
    float value = 0.0f;
    bool valid = false;
};

// 2. message_center.h/.cpp：声明并唯一地定义通道
extern Topic<ExampleState> Example_State_Topic;
Topic<ExampleState> Example_State_Topic;

// 3. 发布端和订阅端绑定同一实例
static Publisher<ExampleState> publisher(MessageCenter::Example_State_Topic);
static Subscriber<ExampleState> subscriber(MessageCenter::Example_State_Topic);
```

新增后需要在本文“当前静态通道”表中记录所有权，并验证首次读取、多订阅者独立新数据、
sequence 回绕假设和消息体积。

## 10. 新增 EventQueue

先确定容量依据和满队列策略。当前实现固定采用“拒绝新元素并计数”，不支持覆盖最旧值。

```cpp
enum class ExampleEventType : uint8_t { Start, Stop };
struct ExampleEvent { ExampleEventType type; };

extern EventQueue<ExampleEvent, 4U> Example_Event_Queue;
```

生产者必须处理 `Push()==false`；消费者必须规定每周期最多处理几个事件、禁用状态如何
处置排队事件，以及连续控制模式与事件的优先级。

## 11. 边界与禁止事项

Message Center 不负责：

- CAN/UART/USB/板间 Transport。
- Device 在线检测和离线动作。
- Safety Manager、日志、参数、硬件容器或自动代码生成。
- 动态 Topic 创建、字符串查找、订阅注销或跨板路由。

禁止重新引入：

- 运行时 Publisher/Subscriber 注册表。
- 每订阅者一个 FreeRTOS Queue 的 Latest-Value 模拟。
- `malloc/new`、链表、map 或字符串 Topic 名。
- 在 Message Center 中直接包含 Gimbal、Chassis、Shoot 控制实现。

## 12. 验证清单

- Topic 首次发布前 `Read()` 返回 false 且不改输出。
- `ReadWithMeta()` 的数据、sequence、timestamp、valid 来自同一快照。
- 两个 Subscriber 能分别读取同一次发布，单个 Subscriber 不重复报告。
- EventQueue 保持 FIFO；空队列 Pop 失败；第 N+1 次 Push 失败且 overflow 加一。
- 全仓只有 `message_center.cpp` 定义业务通道实体。
- 不存在 DynamicMessageCenter、字符串 Topic 或 FreeRTOS Queue 依赖。
- INS → Gimbal 和 ControlTask 链路没有增加任务或阻塞点。

## 13. 相关文档

- [框架总览](../../../README.md)
- [BSP](../../Middleware/BSP/README.md)
- [Application](../../Application/README.md)
- [交互式架构图](../../../Assets/Architecture/H7_BSP.html)

# Application 层开发指南

Application 层描述机器人“要做什么”，按机械功能组织 Gimbal、Chassis、Shoot 和命令
所有者 RobotCmd。FreeRTOS Task 只负责调度；设备协议、总线缓冲和 HAL 细节分别属于
Device 与 BSP。

本文单独维护应用架构，不把具体机器人控制逻辑混入 BSP 总览。

## 1. 分层边界

Application 可以：

- 保存控制目标、反馈缓存和应用状态。
- 组合 PID、轨迹、滤波等算法。
- 初始化并直接控制自己拥有的 Device。
- 通过 Message Center 与同级 Application 交换状态、命令和事件。

Application 不应：

- 直接管理 DMA 缓冲、HAL Handle 或中断注册。
- 解析底层 CAN/UART/SPI 协议帧。
- 创建另一套消息总线或用字符串查找 Topic。
- 把设备在线检测迁入应用消息中心。
- 在多个模块中争用同一电机或同一命令所有权。

## 2. 当前目录

| 模块 | 职责 | 拥有/调用的主要对象 |
| --- | --- | --- |
| `RobotCmd` | 统一接收上层输入、发布应用命令、汇总反馈 | Message Center Publisher/Subscriber |
| `Gimbal` | 云台模式、目标角/速度、QD4310 控制和反馈 | Yaw/Pitch QD4310、PID、INS Topic |
| `Chassis` | 四舵轮运动学、最短转向和电机目标 | 8 个 DJI 电机及电机组 |
| `Shoot` | 摩擦轮、拨弹连续模式和离散射击动作 | 3 个 DJI 电机、ShootEvent FIFO |
| `Communication` | 当前通信应用骨架 | 后续外部输入或传输接入点 |

硬件路径由 `H7_APP_GIMBAL`、`H7_APP_CHASSIS`、`H7_APP_SHOOT` 控制。默认关闭的模块
仍保留消息端点和反馈结构，但不会访问对应电机硬件。

## 3. Control_Task 生命周期

`Control_Task` 是 Application 的统一 1 kHz 调度入口：

```text
任务启动
  ├─ Gimbal_Init()      条件编译启用时初始化
  ├─ Chassis_Init()
  ├─ Shoot_Init()
  └─ RobotCmd_Init()    装载安全默认命令

每次 1 ms 线程标志
  ├─ RobotCmd_Update()  读取反馈，发布 dirty 命令
  ├─ Gimbal_Update()    读取 INS/命令，执行闭环，发布反馈
  ├─ Chassis_Update()   读取命令，计算轮组目标，发布反馈
  └─ Shoot_Update()     读取连续命令/事件，控制发射，发布反馈
```

顺序是契约：RobotCmd 先发布，消费者在同一控制周期读取；各 Application 更新后发布的
反馈由 RobotCmd 在后续周期读取。Application 不创建额外控制任务。

## 4. RobotCmd：命令唯一入口

RobotCmd 不直接访问电机、CAN 或 IMU。上层输入模块通过以下 API 更新目标：

```cpp
void RobotCmd_SetGimbal(const GimbalCmd &command);
void RobotCmd_SetChassis(const ChassisCmd &command);
void RobotCmd_SetShoot(const ShootCmd &command);
bool RobotCmd_PushShootEvent(const ShootEvent &event);
```

连续命令写入本地缓存并设置 dirty 标志，`RobotCmd_Update()` 才发布到对应 Topic。多个
上层输入若可能同时写同一类命令，必须在 RobotCmd 之前定义优先级和仲裁，不能绕过
RobotCmd 直接发布。

启动默认值：

- Gimbal 为 `LOCK`，避免第一帧目标到达前跳向零点。
- Chassis 为 `ZERO_FORCE`。
- Shoot 总开关、摩擦轮和拨弹盘均关闭。

反馈读取 API 在对应 Application 首次发布前返回 false，并保持调用者输出不变。

## 5. Gimbal

### 5.1 数据输入

- `INS_State_Topic`：Yaw/Pitch/Roll 和机体系角速度。
- `Gimbal_Command_Topic`：目标角、前馈角速度和模式。

### 5.2 模式

| 模式 | 行为 |
| --- | --- |
| `DISABLED` | 关闭 Yaw/Pitch 输出 |
| `IMU` | 使用命令目标与 INS 状态执行闭环 |
| `LOCK` | 锁住当前姿态或给定姿态 |

Yaw 使用角度外环与速度内环，速度内环输出电流命令；Pitch 当前使用 QD4310 内置位置
环。机械角限制在下发前再次约束，避免手动设置绕过范围。

初始化最多等待电机使能 2 秒；超时保留明确错误状态并返回，不能阻塞其他 Application
初始化。成功时读取最新 INS Yaw 和 Pitch 电机角度作为锁定目标。

### 5.3 反馈

控制每 1 ms 更新，`GimbalFeedback` 每 10 个周期发布一次，包含姿态、角速度、INS
有效性和电机使能状态。

## 6. Chassis

当前底盘模型为四舵轮 AGV：

1. 将底盘 `vx/vy/wz` 分解为四个轮模块的平移速度向量。
2. 由 `atan2` 得到目标舵向。
3. 舵向误差超过 90° 时翻转轮速，缩短舵电机转动路径。
4. 轮电机走速度环，舵电机走角度外环。

`ZERO_FORCE` 会关闭轮组和舵向组输出；其他模式使能设备并计算目标。机械尺寸、轮径、
舵向零位和 PID 参数均为实车相关配置，启用前必须标定。

反馈由四个轮模块估算 `vx/vy/wz`，经过一阶平滑后每 10 ms 发布。`online` 只有八个
电机均在线时为 true；在线状态来源仍是 Device/Daemon，而不是 Message Center。

## 7. Shoot

发射控制分为连续状态和离散事件。

### 7.1 ShootCmd

- `ShootMode`：发射机构总开关。
- `FrictionMode` 与 `friction_speed_deg_s`：摩擦轮持续状态。
- `LoaderMode::STOP/REVERSE/BURST`：拨弹盘持续模式。
- `loader_speed_deg_s` / `shoot_rate_hz`：持续目标。

### 7.2 ShootEvent

`ShootOnce` 和 `ShootTriple` 必须通过 `RobotCmd_PushShootEvent()` 进入容量 8 的 FIFO。
相同事件连续 Push 两次代表两个动作，不得改成 Latest-Value 状态。

STOP 模式每周期最多消费一个事件：首次事件从当前反馈角建立目标，后续排队事件在已有
目标上累加 1 或 3 个弹位。BURST/REVERSE 取消事件角度保持；OFF 禁用输出并排空当前
队列，避免重新使能后补射。

调用者必须检查 `RobotCmd_PushShootEvent()` 返回值。返回 false 表示队列已满，本次动作
没有被接受。

## 8. Message Center 使用规则

| 数据 | 通道 | 示例 |
| --- | --- | --- |
| 最新状态 | `Topic<T>` | INS、GimbalFeedback |
| 连续目标 | `Topic<T>` | ChassisCmd、ShootCmd |
| 不可覆盖事件 | `EventQueue<T,N>` | ShootOnce、ShootTriple |
| Application 控制所属 Device | 直接调用 | 电机组 `Control()` |
| 在线状态 | Daemon | 电机反馈 Feed / StatusTask CheckAll |

完整接口、并发和通道所有权见 [Message Center 文档](../System/MessageCenter/README.md)。

## 9. 添加新的 Application

1. 按机械功能建立目录和公开头文件，明确它拥有的 Device。
2. 定义初始化函数和单周期 Update；构造函数中不访问硬件。
3. 确定输入输出语义，并在 Message Center 中声明唯一静态通道。
4. 连续命令由唯一所有者发布；离散动作明确容量、溢出和禁用策略。
5. 在 `Control_Task` 中按数据依赖安排调用顺序，不轻易新增任务。
6. 用 CMake 选项控制尚未标定的硬件路径，默认状态必须安全。
7. 更新本文、Message Center 通道表、根 README 和架构图。

推荐接口形态：

```cpp
bool Example_Init(void);
void Example_Update(void);
```

若初始化失败，调用方必须能够继续初始化其他应用；Update 必须在设备未就绪时安全返回。

## 10. 实时性与错误处理

- Control_Task 使用 High1 优先级，BMI088 解算使用 High2；不要在控制周期等待 I/O。
- 反馈降频只减少跨模块复制，不改变设备控制频率。
- 电机/CAN 提交返回 false 时保留失败状态或执行明确重试策略。
- 不在控制循环动态分配内存、创建容器或进行无界遍历。
- 不在 Application 直接操作 DMA 缓冲或依赖回调参数超出其有效期。
- 所有角度、速度、时间和控制量必须注明单位。

## 11. 启用前检查

### Gimbal

- FDCAN 总线、节点 ID、机械范围和方向。
- QD4310 模式、使能反馈和 Yaw/Pitch PID。
- INS 坐标系、角度符号和零位。

### Chassis

- 八个电机的总线、ID、方向和在线状态。
- 轮径、半长、半宽和四个舵向零位。
- 零速保持、最大速度和实际运动方向。

### Shoot

- 摩擦轮方向、目标速度和拨弹盘减速比。
- 单弹角度、三连发累加方向和角度环限幅。
- OFF、BURST、REVERSE 与排队事件的实机安全行为。

## 12. 开源适配

Application 边界、四舵轮运动学和基础发射控制参考 Meta-Embedded-NG，并适配为本工程
的 C++ Device、CMSIS-RTOS v2、静态 Message Center 和 CAN 提交语义。许可信息见
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。

## 13. 相关文档

- [框架总览](../../README.md)
- [BSP](../Middleware/BSP/README.md)
- [Message Center](../System/MessageCenter/README.md)
- [交互式架构图](../../Assets/Architecture/H7_BSP.html)

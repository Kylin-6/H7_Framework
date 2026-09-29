# Application 层开发指南

Application 层描述机器人“要做什么”，按机械功能组织 Gimbal、Chassis、Shoot 和命令
所有者 RobotCmd。FreeRTOS Task 只负责调度；设备协议、总线缓冲和 HAL 细节分别属于
Device 与 BSP。

本文单独维护应用架构，不把具体机器人控制逻辑混入 BSP 总览。

## 物理单位

Application、Algorithm、Message Center 和 Transport 的物理量统一使用 SI：角度
`rad`、角速度 `rad/s`、线速度 `m/s`、转矩 `N·m`。时间接口保留明确的 `s/ms/us`
后缀。机械标定表可用角度书写，但必须在常量定义处通过 `DegToRad()` 转成 rad；
调试显示和外部协议可保留 degree，进入控制链前须转换。DJI 的编码器、RPM、原始
电流和 CAN 指令仍按设备协议处理；Application 仅使用输出侧 rad/rad/s 反馈。

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
| `RobotCmd` | 命令唯一所有者和发布者；输入互锁失效时立即发布安全目标 | Output、ShootEvent FIFO、反馈 Topic getter |
| `Gimbal` | 云台模式、目标角/速度、达妙控制和反馈 | 两轴 Class_DMMotor、Yaw PID、INS Topic |
| `Chassis` | 四舵轮运动学、最短转向和电机目标 | 8 个 DJI 电机及电机组 |
| `Shoot` | 摩擦轮、拨弹连续模式和离散射击动作 | 3 个 DJI 电机、ShootEvent FIFO |
| `Communication` | UART5 S.BUS 适配、固定来源输入状态与仲裁 | S.BUS、InputState |

Chassis 与 Shoot 的机械参数和 PID 初值分别放在 `Chassis_Config.h`、`Shoot_Config.h`；
运行状态和设备实例由各自 `.cpp` 内的私有 Context 持有。BoardConfig 只提供总线等硬件
资源，不存机构参数。Gimbal 保持现有 `Gimbal_Config.h` 与状态机组织，本轮不拆文件。
`Communication` 目录本轮保留；老步兵分支合并稳定后可考虑改名为 `Input`。

单板固件的硬件路径由 `H7_APP_GIMBAL`、`H7_APP_CHASSIS`、`H7_APP_SHOOT` 控制，默认均关闭；
双板固件由 CMake 在构建期分别选择应用和任务源码。板内命令通过 `LocalPublisher` 进入
Message Center，云台板的底盘命令通过 `RemotePublisher` 进入固定 CAN Transport。

## 3. Control_Task 生命周期

各板的 `Control_Task` 均由 1 ms 线程标志唤醒，当前初始化和更新顺序为：

```text
SingleBoard: RobotCmd_Init → Communication_Init → Gimbal_Init(启用时) → Chassis_Init → Shoot_Init
             Communication_Update → RobotCmd_Update → Gimbal_Update → Chassis_Update → Shoot_Update
GimbalBoard: BoardTransport_Init → RobotCmd_Init → Communication_Init → Gimbal_Init → Shoot_Init
             BoardTransport_Poll → Communication_Update → RobotCmd_Update → Gimbal_Update → Shoot_Update
ChassisBoard: BoardTransport_Init → Chassis_Init
              BoardTransport_Poll → Chassis_Update
```

RobotCmd 初始化失败时控制任务停在延时循环；不会继续初始化电机应用。RobotCmd 在
消费者之前发布命令；反馈 getter 在调用时直接读取各 Application 的 Topic。
Gimbal/Chassis 板间轮询复用该任务，不创建额外控制任务。

## 4. RobotCmd：命令唯一入口

RobotCmd 不直接访问电机、CAN 或 IMU。输入链现在是：

```text
UART5 S.BUS → Communication_Update → InputState(Remote)
VTM / Keyboard / Vision → InputState_Submit*（接入接口，当前未绑定设备）
InputState → SourceArbitration_Resolve → RobotCmd_Update → Output
```

`InputState` 是固定的四份状态，不是动态 Topic 路由。Remote 为安全许可：最近 50 ms
内必须有健康帧且通过 200 ms 解锁去抖。默认选择 Remote；只有遥控模式切换逻辑显式
调用 `InputState_Select()` 后，才选择 VTM 或 Keyboard。新来源必须在切换后提供新鲜
命令（100 ms 门限），否则进入安全态，不自动回退。Vision 只有在显式允许、目标新鲜
且当前来源允许云台控制时，才覆盖云台角目标；目标失效后云台进入 `LOCK`。
VTM、Keyboard、Vision 当前没有绑定 UART/协议，生产固件不会自动选择或启用它们。

以下设置接口保留供已有调用方使用；控制任务运行时，仲裁结果在每次
`RobotCmd_Update()` 中覆盖缓存目标：

```cpp
void RobotCmd_SetGimbal(const GimbalCmd &command);
void RobotCmd_SetChassis(const ChassisCmd &command);
void RobotCmd_SetShoot(const ShootCmd &command);
bool RobotCmd_PushShootEvent(const ShootEvent &event);
```

连续命令写入本地缓存并设置 dirty 标志，`RobotCmd_Update()` 才通过已注入的 Output
发布；底盘命令每 10 ms 刷新。Setter 当前没有并发保护，应由 ControlTask 上下文调用，
不能直接从 ISR/UART 回调并发修改。S.BUS 驱动只在 UART 中断保存完整帧，
Communication 在 ControlTask 中读取快照并提交 Remote 输入；VTM/键鼠/Vision 的
未来适配器同样必须在任务上下文提交状态。

S.BUS 使用 UART5：帧新鲜度 50 ms，frame-lost/failsafe 立即锁定；连续 200 ms 健康且
CH1–CH4 回中后解锁。CH2/CH1 映射底盘前后/左右，CH7 为速度档，CH10 负半轴为手动旋转；
CH5 跟随、CH3/CH4 云台与 CH6 发射暂未接入。旧步兵的 30/50 非 SI 参数不移植，
目前调试上限为 0.5 m/s 和 1 rad/s，实车使用前须确认方向、机械零位与限幅。
失联时清除未执行的发射事件，并立即发布 Gimbal `DISABLED`、Chassis `ZERO_FORCE`、Shoot `OFF`。

RobotCmd 独立初始化时的默认值如下；ControlTask 随后初始化 UART5 输入互锁，
在 S.BUS 解锁前把云台覆盖为 `DISABLED`：

- Gimbal 为 `LOCK`，避免第一帧目标到达前跳向零点。
- Chassis 为 `ZERO_FORCE`。
- Shoot 总开关、摩擦轮和拨弹盘均关闭。

三个反馈 getter 都要求对应 Application 最近 100 ms 内发布；未发布或超时均返回
false，并保持调用者输出不变。RobotCmd 不再维护应用反馈的二次缓存。

## 5. Gimbal

### 5.1 数据输入

- `INS_State_Topic`：Yaw/Pitch/Roll 和机体系角速度。
- `Gimbal_Command_Topic`：目标角、前馈角速度和模式。

### 5.2 模式

| 模式 | 行为 |
| --- | --- |
| `DISABLED` | 关闭 Yaw/Pitch 输出 |
| `IMU` | 使用命令目标与 INS 状态执行闭环 |
| `LOCK` | 捕获并保持当前姿态，忽略命令目标字段 |

Yaw 使用 INS 角度/速度串级闭环，通过达妙 MIT 纯转矩指令输出 N·m；Pitch 将 INS
姿态误差转换为 MIT 电机位置/速度目标，并限制机械范围。两轴预先配置为 MIT 模式。

初始化仅校验配置和注册驱动。使能、两秒超时、一秒退避和自动恢复均由 Update
非阻塞推进；恢复需反馈持续有效 100 ms，随后重置控制器并捕获当前姿态。IMU 模式
需在恢复后发布新目标，避免旧目标重放；DISABLED 或故障时清零两轴 MIT 输出并重试失能。

配置集中在 [Gimbal_Config.h](Gimbal/Gimbal_Config.h)。默认关闭云台编译选项；Yaw 转矩环
增益全零，Pitch 增益和限位来自参考机构示例，均须实机标定。完整公式、参数来源、
状态语义和测试见 [双达妙云台说明](Gimbal/README.md)。

### 5.3 反馈

控制每 1 ms 更新，`GimbalFeedback` 每 10 个周期发布一次，包含姿态、角速度、INS
有效性和电机使能状态。

## 6. Chassis

当前底盘模型为四舵轮 AGV：

1. 将底盘 `vx/vy/wz` 分解为四个轮模块的平移速度向量。
2. 由 `atan2` 得到目标舵向。
3. 舵向误差超过 90° 时翻转轮速，缩短舵电机转动路径。
4. 轮电机走速度环，舵电机走角度外环。

轮速目标按 `omega_rad_s = velocity_m_s / wheel_radius_m` 计算；反馈按
`velocity_m_s = output_speed_rad_s * wheel_radius_m` 还原。舵向零位在标定处由角度
转成 rad，`atan2`、`remainder`、`sin/cos` 均直接使用 rad。轮电机速度环输出为
协议电流原始值；舵向角度环输出为速度目标 rad/s，原 200/1000 deg/s 的积分/输出
限幅已分别转成约 3.49/17.45 rad/s。

`ZERO_FORCE` 会关闭轮组和舵向组输出；其他模式使能设备并计算目标。机械尺寸、轮径、
舵向零位和 PID 参数均为实车相关配置，启用前必须标定。
`NO_FOLLOW`、`FOLLOW_GIMBAL_YAW`、`ROTATE` 枚举已定义，当前没有彼此独立的控制分支。
舵向使用 `output_total_angle`，上电绝对零位不能仅由增量编码器确定；实车需要可靠的
绝对编码器、寻零或已知上电姿态。车体 `vx/vy/wz` 的物理正方向尚待接线和坐标标定。

现有轮速环、舵向角度环和舵向速度环增益没有可靠的实车单位/整定记录。它们目前
只作为初始占位值，启用电机前必须按 rad/rad/s 反馈重新整定；不能把旧的混合单位
表现视为等效基线。

反馈由四个轮模块估算 `vx/vy/wz`，经过一阶平滑后每 10 ms 发布。`online` 只有八个
电机均在线时为 true；在线状态来源仍是 Device/Daemon，而不是 Message Center。

## 7. Shoot

发射控制分为连续状态和离散事件。

### 7.1 ShootCmd

- `ShootMode`：发射机构总开关。
- `FrictionMode` 与 `friction_speed_rad_s`：摩擦轮持续状态。
- `LoaderMode::STOP/REVERSE/BURST`：拨弹盘持续模式。
- `loader_speed_rad_s` / `shoot_rate_hz`：持续目标。

摩擦轮 M3508/C620 使用直连转子、无原厂减速箱，明确配置 `gear_ratio = 1.0`，
默认目标为 `25 rad/s`。型号仍决定 CAN 协议、ID、电流指令范围及温度反馈。
单弹 36°、反转 -360 deg/s 作为机械标定输入后分别
转换为约 0.62832 rad、-6.28319 rad/s。发射反馈字段均为输出轴 rad/rad/s。
拨弹角度环输出为速度目标，原 360 deg/s 限幅已换算为 `2π rad/s`。
摩擦轮和拨弹速度环增益尚无可靠实车整定记录，必须重新实车整定。

### 7.2 ShootEvent

`ShootOnce` 和 `ShootTriple` 必须通过 `RobotCmd_PushShootEvent()` 进入容量 8 的 FIFO。
相同事件连续 Push 两次代表两个动作，不得改成 Latest-Value 状态。

STOP 模式每周期最多消费一个事件：首次事件从当前反馈角建立目标，后续排队事件在已有
目标上累加 1 或 3 个弹位。BURST/REVERSE 取消事件角度保持；OFF 禁用输出并排空当前
队列，避免重新使能后补射。
每次成功 Push 只代表一个逻辑动作请求；事件按目标角累加，不等待前一发物理完成。
当前没有摩擦轮就绪、卡弹检测/回退、热量限制、裁判系统互锁或完整 FEEDING 状态机。

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

机械参数放 Application 的 Config，运行状态和设备实例放私有 Context；公共接口放 `.h`，
控制细节和私有 helper 留在 `.cpp`。Task 只安排初始化顺序和周期调用。

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
- 两轴达妙 MIT 模式与协议量程、Yaw 转矩 PID、Pitch MIT 增益及限位。
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

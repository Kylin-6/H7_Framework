# H7_BSP RoboMaster 框架审查与发展建议

审查日期：2026-09-25。审查对象：本地 H7_BSP `fac0977`；参考：Meta-Embedded-NG `a0ebb95`、basic_framework `1a136eb`。结论针对这三个本地版本，不代表上游最新版本。

## 1. 总体判断

**H7_BSP 已经具备较扎实的 H7 板级基础，但整车应用仍处于接入、移植与验证阶段。下一阶段最有价值的工作是修复应用与驱动之间的契约错误，形成可验证的安全控制链，再扩充功能。**

底层已有明确的 DMA 内存布局、CAN 命令/连续量分流、静态类型化消息中心、带时间戳的在线检测及主机回归测试。这些应保留。主要短板是：

- 底盘、发射应用使用的角度单位与 DJI 驱动接口不一致，影响实际控制量。
- 裁判系统解析器存在已复现的越界读取、循环无法退出和拆包丢帧。
- 命令新鲜度、急停、执行确认、故障恢复尚未形成整车闭环。
- 应用模式、输入接入、比赛资源约束和配置管理不完整。
- 当前测试主要验证基础组件，尚未覆盖最容易出问题的应用组合。

默认三个硬件应用均关闭，因此应用缺陷**不等于默认固件已经在驱动电机时触发**；但在启用相应应用前必须处理。裁判系统已有解析入口，当前启动链未接入 `RefereeInit()`，其问题同样属于接入前阻断项。

### 审查方法与边界

本次人工检查了启动、任务调度、消息中心、CAN/UART/OSPI、主要电机驱动、遥控/裁判协议及云台/底盘/发射调用链，并对照参考工程的对应模块。执行了主机回归、两种固件构建及解析器定向复现。

未连接实物，未进行 CAN 抓包、DMA 压力、整车运动、功率、温升或最坏执行时间测试。下文明确区分源码可确认的问题与需实机验证的风险。没有修改生产代码。

## 2. 实际验证结果

| 检查 | 结果 | 能说明什么 |
| --- | --- | --- |
| Boundary / CAN / FilterPolynomial / Fuzzy / SBUS / Topic / Trajectory | 7 个工程、19 个 CTest 测试全部通过 | 已有组件回归基线可用 |
| Debug 默认配置 | 构建成功 | 默认固件可编译、链接 |
| Debug，Gimbal/Chassis/Shoot 全开启 | 构建成功 | 硬件应用代码可以编译，不代表控制正确 |
| 裁判系统短载荷帧，真实解析器 + CRC 实现 + ASan | 发现 heap-buffer-overflow | CRC 正确仍不能保证按 CmdID 复制安全 |
| 裁判系统长度回绕帧 | 2 秒未返回，由复现程序终止 | 特定长度使解析偏移不前进 |
| 同一合法裁判帧整体输入/拆成两段输入 | 整体 `init_flag=1`；拆分 `init_flag=0` | 当前解析器不能跨 DMA 回调保留半帧 |

使用本机 GNU Arm 16.2.0、CMake/Ninja，固件为 Debug 构建。编译存在告警，不能称为“零告警构建”。

| 链接区域 | 默认配置 | 全应用开启 | 解释 |
| --- | --- | --- | --- |
| DTCMRAM / 128 KiB | 107568 B，82.07% | 116072 B，88.56% | 含静态预留的 RTOS heap 等，不等同于运行时堆使用率 |
| RAM_DMA / 64 KiB | 20704 B | 20704 B | DMA 专用区已有独立布局 |
| RAM_D1 / 256 KiB | 16384 B | 16384 B | 主要包含预留的另一段 RTOS heap |
| FLASH / 1 MiB | 121988 B | 130916 B | Flash 尚有较大空间 |

首次审查的构建、测试及复现文件曾保存在 `/tmp/h7-framework-review`；2026-09-26 补充检查时该临时目录已不存在。上表记录首次审查实际结果，长期跟踪时应将相应用例纳入正式 Tests。

## 3. 与两个参考框架的对照

| 维度 | H7_BSP 当前情况 | 可参考的实现 | 建议 |
| --- | --- | --- | --- |
| 模块通信 | 静态 `Topic<T>` + 独立事件 FIFO，有元信息与新鲜度接口 | basic 的字符串话题、动态订阅队列体现模块解耦思想 | 保留 H7 静态实现，补业务新鲜度与所有权 |
| 整车命令 | RobotCmd 主要缓存、转发，没有真实输入仲裁链 | basic 的遥控/键鼠选择、EmergencyHandler；Meta 的 dt7/vtm/auto 命令模块 | 补输入到安全状态再到命令的完整链路 |
| 车型配置 | 三个应用开关，参数散落在源码 | Meta 的 `ROBOT=sentry/infantry`、独立参数头 | 采用明确的机器人配置档 |
| 比赛功能 | 基础运动、基础发射，缺少资源约束集成 | basic 的功率控制/超容接入；Meta 的发射状态机 | 借鉴职责划分，重新标定和测试参数 |
| 单板/双板 | 当前主要是单板应用消息 | basic 的板角色、CANComm 接入 | 有双板需求时增加显式传输适配 |
| 工程验证 | 已有较好的主机算法与边界测试 | 两个参考快照未提供同等主机测试体系 | 以 H7 测试基线向应用与故障注入扩展 |

参考依据：

- [Meta 车型选择与消息契约](/home/kylin6/code/project/rm/opensourse/Meta-Embedded-NG/application/robot_def.h:8)、[Meta 构建选择](/home/kylin6/code/project/rm/opensourse/Meta-Embedded-NG/Makefile:267)。
- [basic 命令组织和急停](/home/kylin6/code/project/rm/opensourse/basic_framework/application/cmd/robot_cmd.c:297)、[basic 单双板配置](/home/kylin6/code/project/rm/opensourse/basic_framework/application/robot_def.h:17)。
- [basic 功率限制接入](/home/kylin6/code/project/rm/opensourse/basic_framework/application/chassis/chassis.c:200)、[Meta 发射就绪与热量状态机](/home/kylin6/code/project/rm/opensourse/Meta-Embedded-NG/application/shoot/ammo_booster.c:380)。
- [basic 动态消息中心](/home/kylin6/code/project/rm/opensourse/basic_framework/modules/message_center/message_center.c:33)。

**参考项目不是正确性标准。** basic 的 EmergencyHandler 仍有离线判断 TODO；Meta 的部分机器人初始化/任务调用被注释，发射参数也依赖具体机构。应借鉴完整的职责链与配置组织，不能把拷贝源码视为验证完成。

## 4. 不足清单：优先修复的正确性与可靠性问题

优先级定义：**P1** 为相应功能启用前应解决的问题；**P2** 为近期工程化工作。这里不把所有问题都称为已发生的实机故障。

### 01｜P1：底盘与发射应用混用了 degree 和 radian

**证据充分，源码可直接确认。**

DJI 驱动明确要求角度目标/反馈为 rad、速度目标/反馈为 rad/s，并另行提供 degree 派生字段。但：

- 底盘把 `feedback.output_total_angle` 当作 degree，与 degree 舵向零位相减；轮速目标使用 `360 / 周长` 换算，随后直接传给以 rad/s 为输入的闭环。
- 底盘速度反馈把 rad/s 乘以 `周长 / 360`，造成量纲错误。
- 发射的单弹目标在 rad 反馈上直接累加 `36.0f`，实际表示增加 36 rad，而非设计意图中的 36°（约 0.6283 rad）。
- 摩擦轮/拨弹速度以 deg/s 命名，却直接送入 rad/s 电机接口；反馈也把 rad 数值直接写入带 `_deg` 后缀的字段。

对相同物理角速度，目标数值的 degree/radian 比值约为 **57.3**；这不能靠“实车重新调 PID”解决。实际转速受输出饱和与机构限制，不能据此断言一定跑到 57.3 倍速度。

**建议：** 内部控制统一 SI 单位；在配置和外部显示边界显式转换。同步复核 PID 增益、输出上限、减速比和遥测单位。

**验收：** 1 m/s、轮半径 0.058 m 时应生成约 17.241 rad/s；单发增加约 0.6283 rad；运动学正解/逆解与 degree 遥测转换均有测试。

依据：[DJI 单位契约](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/Device/Peripheral/Motor/DJImotor/dji_motor.h:64)、[反馈计算](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/Device/Peripheral/Motor/DJImotor/dji_motor.cpp:449)、[底盘换算](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/Application/Chassis/Chassis.cpp:115)、[发射累加与反馈](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/Application/Shoot/Shoot.cpp:121)。

### 02｜P1：裁判系统解析存在越界读取和不前进循环

**已通过独立主机程序复现。**

`JudgeReadData()` 验证的是帧声明长度与 CRC，随后按 CmdID 固定复制结构体，未验证声明载荷是否达到该 CmdID 所需长度。CRC 正确但 DataLength=0、CmdID=game_status 的 9 字节帧，会在复制 11 字节 GameState 时越界。

另一个问题是帧总长度强转 `uint16_t`：DataLength=65527 时，加上 9 字节开销回绕到 0，后续 `read_offset += frame_len` 不前进。使用 5 字节合法 CRC8 帧头即可复现无法返回。实际 UART 回调在中断上下文执行解析，接入后该路径可能拖住系统。

VTM 的 0xA5 分支也存在相同的长度强转/偏移模式，本次没有单独动态复现 VTM。

**建议：** 用较宽无符号类型计算长度，先限制协议最大载荷；按 CmdID 校验载荷长度后再复制；CRC 失败逐字节重同步；保证每轮“前进、等待更多数据或退出”之一成立。

**验收：** ASan/UBSan 覆盖短载荷、极大长度、CRC 错误和随机噪声，解析函数有明确单次工作上限。

依据：[裁判长度与复制](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/Device/Peripheral/Referee/referee_26.c:55)、[偏移推进](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/Device/Peripheral/Referee/referee_26.c:138)、[VTM 同类代码](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/Device/Peripheral/Referee/vtm_26.c:59)。

### 03｜P1：裁判、图传、DBUS 的流式接收契约不统一

裁判/VTM 收到不完整帧时直接退出，没有跨回调累积缓冲；DBUS 直接取本次回调最后 18 字节，不保留前次半帧。UART IDLE/DMA 的一次回调并不应被上层等同于一条完整协议帧。

裁判拆包丢帧已经复现。相比之下，现有 SBUS 已实现流式累积、重同步及相关测试，可作为本仓库内的设计参考。

**建议：** 为各协议保留有界接收状态，支持拆包、粘包、噪声与丢字节重同步；复杂解析可移到任务上下文，但不必为每个设备新建线程。

**验收：** 合法帧在每个字节位置切分、任意组合回调后，得到相同解码结果；错误帧之后能恢复。

依据：[裁判解析入口](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/Device/Peripheral/Referee/referee_26.c:33)、[DBUS 回调](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/Device/Peripheral/Remote/remote_control.c:108)、[SBUS 流式实现](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/Device/Peripheral/Remote/sbus.cpp:106)。

### 04｜P1：控制命令没有来源存活与失效策略

RobotCmd 只在 dirty 时发布，应用没有新命令就继续使用上一帧。INS 已使用 10 ms 新鲜度限制，但底盘、发射及云台命令没有相应期限。

一旦未来接入的遥控/上位机输入停止更新，而电机反馈仍在线，底盘仍可持续执行旧速度，发射仍可持续执行 BURST。设备在线不能证明控制意图仍然有效。

**建议：** 为命令入口定义来源、接收时刻、有效期、优先级和重新使能条件。区分“目标数值未变化”和“来源已经失联”；不要只给 dirty 发布的 Topic 生硬套超时，否则恒定目标也会误过期。

**验收：** 保持恒定合法输入不会误停；中断输入后在配置期限内进入安全状态；重连不会自动恢复旧射击事件。

依据：[RobotCmd 发布](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/Application/RobotCmd/RobotCmd.cpp:87)、[底盘旧命令保持](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/Application/Chassis/Chassis.cpp:238)、[发射旧命令保持](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/Application/Shoot/Shoot.cpp:216)。

### 05｜P1：应用忽略发送失败，失能请求可能只尝试一次

云台在模式边沿调用 `QD4310_Disable()`，忽略返回值，然后无条件更新 `Gimbal_Last_Mode`。若命令 FIFO 满导致提交失败，即使以后再次发布 DISABLED，模式已相同，也不再重试。

底盘和发射同样先更新本地 `Output_Enabled`，再调用电机组 Disable，返回值未上报。这里不能把本地模式切换等同于设备已经停止。

**建议：** 明确区分“期望状态、已提交状态、设备反馈状态”；停止请求失败要保持待处理状态，有界重试并上报。对 CAN 恢复后可能过期的旧控制帧也要有取消/覆盖规则。

**验收：** 注入 FIFO 满、HAL 拒收后，停止仍能在恢复后送达；失能未确认前反馈不得宣称停机完成。

依据：[云台模式处理](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/Application/Gimbal/Gimbal.cpp:314)、[QD 动作提交路径](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/Device/Peripheral/Motor/QDrive/QD4310.cpp:92)、[底盘使能状态](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/Application/Chassis/Chassis.cpp:74)、[发射使能状态](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/Application/Shoot/Shoot.cpp:54)。

### 06｜P1：云台状态机主要覆盖初始化，缺少运行期故障转换与恢复

初始化最多重试 2 秒，能避免无限阻塞，这是优点。但：

- 初始化超时后设为 ERROR，没有运行期重新检查并转换 READY 的路径。
- READY 后控制入口主要检查 FSM 和 INS，未利用已有的 `QD4310_IsHealthy()` 重新检查两轴反馈存活与使能状态。
- INS 失效会给 Yaw 零电流，Pitch 保持原内部位置；这是已有策略，但还没有覆盖电机断线、掉电重连等整机情形。

**建议：** 增加明确的运行/失效/恢复流程；恢复前重新捕获目标、复位控制器状态，并由安全条件决定是否重新使能。

**验收：** 开机缺电机、运行中拔线、电机重启、INS 中断及恢复均有状态和输出断言。

依据：[初始化超时](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/Application/Gimbal/Gimbal.cpp:183)、[运行期守卫](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/Application/Gimbal/Gimbal.cpp:290)、[QD 健康接口](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/Device/Peripheral/Motor/QDrive/QD4310.cpp:148)。

### 07｜P1：Yaw 直接相减，没有处理 ±π 跨界

姿态 Yaw 来自 `atan2f`，云台把该值直接交给普通 PID，PID 使用 `Target - Now`。例如目标 +179°、反馈 -179°，得到约 358° 误差，而不是跨界后的约 -2° 最短误差。

**建议：** 若采用单圈角度控制，显式归一化角度误差；若采用连续累计角控制，在 INS 或适配层展开角度并统一目标定义。机械限位需要单独处理。

**验收：** 覆盖 ±π 两侧、连续多圈、模式切换与重连后零点建立。

依据：[Yaw 产生](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/Middleware/Algorithm/Quaternion/alg_quaternion.h:355)、[云台角环](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/Application/Gimbal/Gimbal.cpp:265)、[PID 误差](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/Middleware/Algorithm/PID/alg_pid.cpp:110)。

### 08｜P1：发射缺少摩擦轮就绪联锁与事件生命周期

`shoot_mode=ON` 时，即使摩擦轮 OFF 或未达到目标速度，BURST 和单发事件仍可驱动拨弹。单发事件在 STOP 模式逐周期消费，没有“本次动作完成、失败或过期”的状态。

此外，BURST/REVERSE 不消费也不清理事件；期间进入队列的事件可能在返回 STOP 后执行。OFF 已清队列，但这只覆盖一种取消场景。

**建议：** 定义摩擦轮稳定就绪、拨弹在线、资源允许等发射条件；为事件定义接收/执行/完成/取消语义、期限及模式转换处理。调试反转应有明确例外策略。

**验收：** 摩擦轮 OFF/未就绪时不进弹；模式切换和离线恢复不会补执行过期事件；队满和执行失败可观察。

依据：[发射执行分支](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/Application/Shoot/Shoot.cpp:73)、[事件消费](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/Application/Shoot/Shoot.cpp:118)、[OFF 清队列](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/Application/Shoot/Shoot.cpp:225)。参考 [Meta 发射状态机](/home/kylin6/code/project/rm/opensourse/Meta-Embedded-NG/application/shoot/ammo_booster.c:380)。

### 09｜P1：三路 CAN 共用动作 FIFO，存在跨总线队首阻塞

全局 `Can_TxPendingMessage` 发送失败后保持队首并立即返回。故障总线上的动作会阻止其他总线的使能/失能动作出队；周期槽仍可独立处理，不能夸大为“三路 CAN 全部停止”。

当前只启用了 RX 新消息通知，未找到项目层 bus-off 状态管理及恢复流程。

**建议：** 动作队列按总线隔离，保留每条总线内的必要顺序；为关键停止动作明确服务优先级、过期和恢复策略；增加每总线故障状态、计数与有界恢复。

**验收：** 一条总线持续发送失败，不阻塞其他总线的停止动作；恢复后不执行已经过期的动作。

依据：[CAN 队首保留](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/Middleware/BSP/CAN/bsp_can.c:464)、[通知配置](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/Middleware/BSP/CAN/bsp_can.c:154)。

### 10｜P1/P2：消息中心之外的共享状态缺少完整并发契约

`Topic<T>` 自身有临界区，但 RobotCmd setter 与 Update 操作同一缓存和 dirty 标志，没有同步，也没有在头文件中限定只可由 ControlTask 调用。若未来 TransportTask 或 ISR 调用 setter，可能读到跨字段不一致的命令，或在发布与清 dirty 之间丢掉更新。

DJI CAN ISR 逐字段写反馈，Application/控制计算直接读取多个字段，没有统一快照。这是跨帧一致性风险，不能因单个 float 读写较短就视为整组反馈一致。

当前没有实际输入调用 RobotCmd setter，因此前一问题属于**接入并发输入时必需解决的接口风险**，不是已观测的数据竞争事故。

**建议：** 明确单写者上下文；外部输入先交给同步邮箱，由控制任务统一消费。电机反馈提供短临界区快照或其他可证明一致的读取接口。

**验收：** 在发布、读反馈、清标志各边界注入更新，既不丢最后一条命令，也不拼接不同帧的字段。

依据：[RobotCmd setter](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/Application/RobotCmd/RobotCmd.cpp:106)、[DJI ISR 更新](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/Device/Peripheral/Motor/DJImotor/dji_motor.cpp:440)、[控制读取反馈](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/Device/Peripheral/Motor/DJImotor/dji_motor.cpp:521)。

## 5. 不足清单：功能完整性与工程化

### 11｜P2：应用接口承诺的模式和数据没有全部实现

Chassis 定义 FOLLOW_GIMBAL_YAW、ROTATE 等模式，但当前 Update 只区分 ZERO_FORCE 与其他模式，没有独立的跟随/旋转策略，也没有云台相对角输入。

GimbalCmd 的速度字段被接收，但 Yaw 目标速度随即被角度 PID 输出覆盖，Pitch 速度未进入当前位置控制路径。外部调用者不能从结构体本身判断哪些字段有效。

**建议：** 为每个模式写清输入、坐标系、控制策略和不支持行为；暂未实现的模式应显式拒绝或从公开契约移除。速度字段若表示前馈，要在明确位置叠加并测试。

依据：[消息定义](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/System/MessageCenter/message_types.h:6)、[底盘模式判断](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/Application/Chassis/Chassis.cpp:247)、[速度覆盖](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/Application/Gimbal/Gimbal.cpp:245)。

### 12｜P1（比赛启用前）：功率、热量、堵转等约束尚未贯通

当前底盘只有运动学和电机控制，没有裁判功率限制、超容策略或相应降级路径。发射源码明确说明未移植热量限制和堵转阈值；裁判协议文件的存在不代表这些数据已进入控制决策。

**建议：** 先定义底盘可用功率和发射可用热量的输入契约，再实现限幅/抑制策略；裁判掉线时采用明确保守值。堵转依据实车电流、速度、时间标定，不能直接抄参考阈值。

**验收：** 对资源不足、裁判离线、机构卡滞分别测试限制输出与恢复；比赛数值以实际采用的规则版本和机构验证为准，本报告不指定赛事限值。

依据：[当前底盘执行](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/Application/Chassis/Chassis.cpp:247)、[发射能力说明](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/Application/Shoot/Shoot.cpp:1)。参考 [basic 功率接入](/home/kylin6/code/project/rm/opensourse/basic_framework/application/chassis/chassis.c:200)。

### 13｜P2：缺少可直接运行的整车输入与通信样例

全仓调用检查未发现生产路径调用 `RobotCmd_Set*`、`RemoteControlInit()`、`SBUS_Init()` 或 `RefereeInit()`；Communication 回调为空，且 Com.cpp 未列入固件源文件。Transport 当前主要输出 IMU 遥测。

**影响：** 即使硬件应用开关打开，框架也不会自然变成可遥控的整车示例；用户仍需自行补输入映射、仲裁和传输接入。

**建议：** 首先提供一个经验证的单板最小整车配置：选定一种遥控输入、接入 RobotCmd、安全停机与状态反馈。按实际需求再做视觉或双板版本。

参考 basic 的 CANComm/板角色组织，但板间协议应显式定义版本、序号、长度、时间有效性和重连行为，不直接把内存结构当稳定线协议。

依据：[空通信入口](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/Application/Communication/Com.cpp:8)、[传输任务](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/Task/TransportTask.cpp:28)、[应用调度](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/Task/Control_Task.cpp:31)。参考 [basic 板间命令](/home/kylin6/code/project/rm/opensourse/basic_framework/application/cmd/robot_cmd.c:351)。

### 14｜P2：1 kHz 是调度意图，尚缺截止时间和过载行为验证

TIM4 用一个线程 flag 唤醒 ControlTask；多个周期到来时 flag 可以合并，不能计数漏掉的控制周期。PID 固定 D_T=0.001，尚未看到控制任务迟到、丢周期或实际周期统计。

CAN 发送通过 RTOS tick 独立唤醒，与 TIM4 不构成严格先后依赖；优先级较高只在任务同时就绪等条件下决定执行顺序。BMI088 High2 任务一次持续排空样本队列，也没有显式每批执行预算。

**建议：** 记录控制周期、最大执行时间、迟到次数和数据年龄；定义错过周期时保持/跳过/降级策略。若需要“本周期算完立即发送”，用明确通知或相位设计建立关系。

**验收：** 满 CAN、持续 UART、IMU 积压及遥测同时运行时，在板测量控制延迟分布与最大值，不能仅凭 osDelayUntil 注释宣称严格周期。

依据：[TIM4 唤醒](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/System/callback/callback.cpp:62)、[ControlTask 等待](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/Task/Control_Task.cpp:43)、[CAN 独立周期](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/Task/CanTxTask.cpp:25)、[IMU 排空循环](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/Task/BMI088_Task.cpp:39)。

### 15｜P2：任务级健康监督、创建失败和崩溃留痕不完整

设备 Daemon 不等同于系统看门狗。HAL IWDG/WWDG 未启用；任务创建结果没有逐项检查；HardFault 主要停在循环，栈溢出 hook 保留任务名并断点，没有完整故障现场持久化/复位策略。

**建议：** 检查关键任务创建结果；聚合控制、采集、通信任务的进度后决定是否喂硬件看门狗；保存复位原因与最小 fault 现场。不要简单新增一个无条件喂狗线程。

**验收：** 人为停住关键任务、耗尽任务分配内存、触发 fault，能进入预期停机/复位流程并保留可读原因。

依据：[任务创建](/home/kylin6/code/project/rm/opensourse/H7_BSP/Core/Src/freertos.c:174)、[HardFault](/home/kylin6/code/project/rm/opensourse/H7_BSP/Core/Src/stm32h7xx_it.c:120)、[看门狗配置](/home/kylin6/code/project/rm/opensourse/H7_BSP/Core/Inc/stm32h7xx_hal_conf.h:64)。

### 16｜P2：车型、接线、机械参数和 PID 混在实现文件中

底盘尺寸、轮径、舵向偏置、总线和电机 ID、PID，以及云台机械范围和发射参数都写在具体 cpp/h 内。三个布尔开关只能表示是否编译硬件路径，不能描述可复现的整车组合。

**建议：** 参考 Meta 的车型选择，先建立简单的编译期配置档，集中管理板角色、设备映射、方向、单位、机械参数和控制器参数；加 CAN ID 冲突与范围检查。暂不需要复杂插件系统或运行时配置语言。

**验收：** 两个配置档可独立构建、输出可辨认的配置标识，切换配置不修改算法源码。

依据：[底盘参数](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/Application/Chassis/Chassis.cpp:30)、[云台接线与范围](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/Application/Gimbal/Gimbal.h:16)、[应用选项](/home/kylin6/code/project/rm/opensourse/H7_BSP/CMakeLists.txt:241)。参考 [Meta 参数选择](/home/kylin6/code/project/rm/opensourse/Meta-Embedded-NG/application/robot_def.h:104)。

### 17｜P2：有 Flash 驱动和分区，尚无完整参数服务，OSPI 错误传播仍薄弱

StorageTask 启动后立即退出；目前未发现 IMU A/B 分区被实际参数读写流程使用。因此“已预留双槽地址”不能当作“参数已支持掉电安全保存”。

同时 OSPI 封装返回 void，忽略 HAL 命令/DMA 启动结果。W25Q 有超时计数与 Busy 清理，但不能据此判断硬件事务已经成功结束或正确恢复。

**建议：** 先把 HAL 错误、超时、Abort/重新就绪和调用方结果贯通，再实现小规模参数服务：版本、长度、CRC、A/B 提交、默认值与回滚。先覆盖电机零位和机械参数等实际需求。

**验收：** 注入 DMA 启动失败、超时和写入中断电；调用有界返回，重启只选择完整有效参数。

依据：[空存储任务](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/Task/StorageTask.cpp:26)、[Flash 分区](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/System/Storage/sys_flash_layout.h:21)、[OSPI 丢弃结果](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/Middleware/BSP/OSPI/bsp_ospi.cpp:101)、[W25Q 超时处理](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/Device/Onboard/W25Q64JV/bsp_w25q64jv.cpp:262)。

### 18｜P2：测试缺少应用组合、协议异常和持续集成入口

现有算法与基础边界测试值得肯定。但没有覆盖此次暴露的应用/驱动单位契约、Gimbal 模式故障、Shoot 联锁、RobotCmd 来源超时及裁判解析。Topic 测试没有覆盖独立 EventQueue，DJI 生产驱动也未纳入现有 Boundary 构建。

Tests 各自独立，根工程没有统一主机测试入口，当前 .github 未见 CI 工作流。编译通过不能发现单位不一致；本次 19 项全绿与发现高优先级缺陷同时成立。

**建议：** 增加统一测试命令和 CI：主机回归 + 协议 sanitizer + 默认/应用配置构建。优先测试真实生产源码的边界与跨层契约，避免只验证桩函数。

**验收：** 上述定向复现成为可自动回归的失败/通过用例；单位错误、停止提交失败和输入失联均能被 CI 拦截。

依据：[Boundary 构建源列表](/home/kylin6/code/project/rm/opensourse/H7_BSP/Tests/Boundary/CMakeLists.txt:10)、[Topic 测试入口](/home/kylin6/code/project/rm/opensourse/H7_BSP/Tests/Topic/CMakeLists.txt:6)、[当前测试说明](/home/kylin6/code/project/rm/opensourse/H7_BSP/README.md:281)。

### 19｜P2：诊断接口分散，尚未形成整车可观测性和资源预算

已有 CAN 失败统计、SBUS 诊断、初始化失败掩码和 SystemView，但当前 Transport 主要输出三个姿态角；未见这些诊断在统一任务中聚合并持续上报。

全应用 Debug 构建 DTCM 已占 88.56%，虽然总 SRAM 还有空间，也不能把其它区余量直接当作 DTCM 可用空间。还缺运行时最小空闲 heap、任务栈高水位、队列峰值、控制迟到的统一记录。

**建议：** 新增低频健康快照，包含固件/配置标识、故障原因、输入/INS 年龄、每总线失败计数、队列占用和任务资源指标；建立链接段预算和在板运行预算。内存迁移应依据测量，不先盲目优化。

依据：[CAN 统计](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/Middleware/BSP/CAN/bsp_can.c:452)、[初始化状态](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/System/Init/Init.cpp:39)、[当前遥测](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/Task/TransportTask.cpp:30)、[heap 布局](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_Config/FreeRTOS_Patch/heap_regions_patched.c:32)。

### 20｜P2：构建模块边界与工具链契约仍不够明确

顶层 CMake 把大部分用户源码和 include 路径直接挂在一个目标上，硬件应用通过宏控制内部代码；缺少可单独编译/测试的应用与平台边界。工程指定 C11，但未显式固定 C++ 标准；主机测试使用 cxx_std_17，固件依赖编译器默认值。

本次 GNU Arm 16.2.0 构建出现 volatile 自增弃用和 PID typedef 等告警；没有固定工具链版本与告警基线，换编译器容易产生不一致预期。

文档也存在具体漂移：Application 文档把设备在线来源概括为 Device/Daemon，但实际 DJI 自行检查超时、QD 按需查询、当前 Daemon 注册出现在 DM 路径，维护者不能假设 StatusTask 覆盖所有电机。

**建议：** 逐步建立少量 CMake target（平台、设备、应用、纯算法），用依赖限制 include；固定 C++ 标准和已验证工具链版本。维护模块“已实现/已接入/已测”的状态表，修正文档中的过度概括。

依据：[顶层构建](/home/kylin6/code/project/rm/opensourse/H7_BSP/CMakeLists.txt:11)、[源码集中注册](/home/kylin6/code/project/rm/opensourse/H7_BSP/CMakeLists.txt:106)、[在线检查实际路径](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/Device/Peripheral/Motor/DJImotor/dji_motor.cpp:489)、[Daemon 调度范围](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/Task/StatusTask.cpp:10)。

## 6. 建议的发展顺序

下面以验收门槛划分阶段，而不是假定人力后给出不可靠工期。

| 阶段 | 核心工作 | 完成标志 |
| --- | --- | --- |
| A：建立正确性基线 | 修复 degree/radian、裁判长度/拆包、Yaw 跨界；补定向回归 | 问题 01/02/03/07 的测试可自动验证，默认及全应用构建通过 |
| B：完成安全控制链 | 来源存活、急停/重新使能、失能重试、云台故障状态、发射联锁、CAN 故障隔离 | 断输入、断电机、队满、单总线故障均得到可断言的输出状态 |
| C：形成一套实车配置 | 遥控接入、机器人配置档、标定、功率/热量约束、健康遥测 | 一份配置可完成启动、控制、停机、恢复全过程；参数与日志可追溯 |
| D：团队维护与扩展 | CI、构建边界、参数持久化、硬件回归、按需求增加双板/视觉 | 新人可按文档构建和接线；增加车型无需复制整套底层代码 |

建议下一轮直接拆为六个独立工作项：

1. **统一 DJI 应用单位并补运动学/发射目标回归。**
2. **重做裁判/VTM 的有界流式解析，纳入 sanitizer 测试。**
3. **为 RobotCmd 定义来源存活、安全状态和重新使能契约。**
4. **贯通停止请求、提交失败、设备确认和恢复流程。**
5. **补云台/发射/底盘应用测试及每总线故障注入。**
6. **建立一个可运行的机器人配置档、统一健康输出和 CI。**

## 7. 应当保留的现有设计

- 保留静态、类型化 Topic；连续状态与离散事件使用不同通道。
- 保留 CAN 连续量覆盖、动作 FIFO 的语义划分，在其上补总线隔离与安全处理。
- 保留明确的 DMA 区域和工程自有链接脚本；不要把参考 F4 的内存假设搬到 H7。
- 保留传感器 ISR 与任务解算分离、统一控制任务和反馈降频。
- 保留已有算法随机测试与边界测试，向应用集成扩展。
- 保留默认关闭未标定硬件应用的做法，并把“编译可用”与“实车已验证”明确区分。

近期不建议继续堆叠控制算法、改写成全新消息总线、为每个模块新建任务或泛化大型设备插件体系。现有基础足以承载后续整车开发，优先把单位、时序、故障和配置契约真正贯通。

## 8. 补充：代码实现层面的优化机会（2026-09-26）

以下针对现有实现的运行成本、可读性与接口设计。源码能证明重复操作存在，但尚未在板测量耗时，不能承诺具体 CPU 降幅。应先修复单位和协议正确性，再比较优化前后的 Release 行为与时序。

| 优先顺序 | 当前位置与问题 | 建议实现 | 收益与约束 |
| --- | --- | --- | --- |
| 1 | DJI 每次 Control 都扫描三组 PID 调参字段，并导出三组详细调试数据 | 控制周期边界应用显式待更新参数；普通遥测按 50–100 Hz 快照；需要高速辨识时单独启用采样 | 降低每电机每周期的非控制工作；保留必要的调试能力，不直接删除 |
| 2 | Chassis 每 1 ms 计算反馈中的四组 sin/cos 和速度估计，但只每 10 ms 发布 | 若只有 100 Hz 应用消费者，可将反馈估计降至所需频率，控制计算保持 1 kHz | 降频后须按新采样周期重算滤波系数；若用于高频观测器则不能直接降频 |
| 3 | Subscriber::Read 先完整复制 TopicSnapshot，再判断序号有没有变化 | 在 Topic 内提供一次临界区完成的 ReadIfNew：先比较序号，新数据才复制 | 对低频命令、高频轮询避免无效载荷复制；不能在临界区外分两次读序号和数据 |
| 4 | CAN_Tx_Perform 在关中断区线性扫描最多 32 个槽；RX 每帧最多检查 16 个回调 | 可在初始化时注册周期发送槽并保存稳定句柄；发送仍在短临界区复制；RX 先按总线缩小表 | 主要改善临界区上界和扩展性；当前容量小，应测量后决定，不急于加入哈希表 |
| 5 | Chassis_NormalizeAngle 反复加减整圈，耗时随累计角增大；无非有限数入口检查 | 在修复单位后，统一使用有限值检查和固定步骤的 wrap 运算，或维持单圈角+圈数表示 | 避免运行时间随圈数增长；验证 ±π 端点语义、负值和大角精度，不能直接声称 libm 一定更快 |
| 6 | DJI Control() 与 Control(ref...) 的“是否发送”语义不同；bool 同时编码 ready 和提交结果 | 区分 Calculate/Submit，或返回具名结果；合并 Control 与 Control_Degree 的共同流程 | 降低误调用与重复实现；对公共 Send 独立调用保留必要的超时防护 |
| 7 | Gimbal 用很长的位置参数列表调用 PID::Init，已有 PID_InitTypeDef 却未形成统一入口；标量普遍以 const float& 传递 | 增加 Init(const PID_InitTypeDef&) 并复用现有校验；小标量参数优先按值 | 主要减少顺序误传和接口维护成本；ABI/调用成本差异需看生成代码 |
| 8 | Matrix 数组有成员零初始化，默认构造又清零；固定数组手写复制/移动操作，移动仍是 memcpy | 在接口兼容前提下使用默认构造/复制/赋值，保留真正有意义的数学操作 | 减少样板代码、改善类型性质；编译器可能已消除重复清零，不算已证明的性能收益 |
| 9 | OSPI 中断/完成链路含 RTT 格式化输出 | ISR 只记录事件码、时间和计数，低频任务格式化输出 | 减少中断工作量；阻塞与否取决于 RTT 配置，不能一概断言会阻塞 |
| 10 | Shoot OFF 先 Size 再逐条 Pop，每个操作单独进临界区 | 若契约就是取消当前全部事件，增加有明确并发语义的 Clear，并在禁用状态拒绝新发射事件 | 简化取消流程；现有容量仅 8，性能收益有限，重点是语义明确 |

### 8.1 具体代码依据

- [DJI 调参同步与调试导出](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/Device/Peripheral/Motor/DJImotor/dji_motor.cpp:288)、[每周期调用](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/Device/Peripheral/Motor/DJImotor/dji_motor.cpp:521)。
- [底盘反馈计算](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/Application/Chassis/Chassis.cpp:150)、[计算与发布频率](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/Application/Chassis/Chassis.cpp:262)。
- [订阅者读取](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/System/MessageCenter/topic.h:168)、[CAN 槽查找](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/Middleware/BSP/CAN/bsp_can.c:341)。
- [累计角归一化](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/Application/Chassis/Chassis.cpp:60)、[电机组接口语义](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/Device/Peripheral/Motor/DJImotor/dji_motor.cpp:749)。
- [PID 初始化长参数列表](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/Application/Gimbal/Gimbal.cpp:135)、[Matrix 构造与复制](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/Middleware/Algorithm/Matrix/alg_matrix.h:31)。
- [OSPI ISR 日志](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/Middleware/BSP/OSPI/bsp_ospi.cpp:159)、[Shoot 清事件](/home/kylin6/code/project/rm/opensourse/H7_BSP/User_File/Application/Shoot/Shoot.cpp:225)。

### 8.2 不建议为了“优化”贸然做的改动

- 不把所有短临界区替换为复杂无锁结构。当前单核、小消息设计可以合理，先测最大关中断时长。
- 不删除 UART/SPI 的拥有型发送缓冲复制。这些复制承担 DMA 生命周期和内存可达性保证。
- 不把全部三角函数换成手写近似、把浮点统一改定点，或给全部代码加 inline。
- 不把小型数组查找一律改成动态容器/哈希表。
- 不直接跳过 PID 的零增益分支状态更新：当前 D 滤波在 Kd=0 时继续跟踪，改变这一点会影响重新启用的瞬态。
- 不简单把 1 kHz 反馈计算挪到 100 Hz 后保留原滤波系数；应保持等价时间常数并回归验证。
- 不仅用 Debug 构建耗时决定优化。对比同工具链、同 Release 配置、相同负载的最大周期耗时、中断屏蔽时间、栈及 RAM 占用。

建议首先做“调试路径降频、底盘反馈按需计算、消息无更新不复制、API 语义明确”四项；之后依据在板测量决定是否引入 CAN 槽句柄或进一步改动数学热路径。

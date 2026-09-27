# 文档与源码注释一致性审查（2026-09-28）

事实来源：`RoboMaster_H7` 基准提交 `7212ce1` 及本次文档注释差异；测试清单取自本地 `RoboMaster_Test/Tests`。本次没有修改生产行为。

## 当前架构与纠正

- CMake 配置期选择 SingleBoard、GimbalBoard、ChassisBoard 的应用、设备和任务源码；BoardConfig 描述硬件，TransportConfig 描述固定跨板 CAN 映射。没有 Router、动态 Topic 或新任务。
- RobotCmd 是命令唯一发布者。SingleBoard 的三个 Output 均为 LocalPublisher；GimbalBoard 的底盘 Output 为 RemotePublisher。GimbalBoard 的 `ChassisCmd` 经 `0x141` 到 ChassisBoard 本地 Topic；ChassisBoard 的 `ChassisFeedback` 经 `0x222` 返回 GimbalBoard 本地 Topic。两者均为 latest-value，100 ms 时效按实际 CAN 接收时刻判定。
- Topic 只保留最新快照，EventQueue 是固定 FIFO；软件提交不代表硬件发送或执行。1 kHz INS/Gimbal/Shoot 板内链不经过 Transport。
- SingleBoard 默认关闭 Gimbal、Chassis、Shoot 电机控制。Remote、Vision 和 Referee 尚未接入 RobotCmd/底盘/发射控制链。

## 修改文件与原因

| 文件 | 同步原因 |
| --- | --- |
| `README.md` | 补当前双板数据流、DM 入队恢复和真实测试套件，移除未跟踪的 VS Code 链接。 |
| `CHANGELOG.md` | 记录已完成多板、反馈、FIFO、Output、SI 和直驱摩擦轮工作；标明旧日期条目的历史语境。 |
| `User_File/Application/README.md` | 修正三目标初始化/更新顺序、Output 注入、命令并发、底盘模式/零位及发射缺口。 |
| `User_File/Application/Gimbal/README.md` | 分清单板与云台板的 DM 总线接线及启用默认值。 |
| `User_File/System/MessageCenter/README.md` | 加入 Output 契约，区分单板/双板 Topic 发布者和订阅者。 |
| `User_File/System/Transport/README.md` | 明确固定协议及序号会话重建边界。 |
| `User_File/Middleware/BSP/README.md` | 解释三路 FIFO 隔离、发送阶段和 UART DMA chunk 边界。 |
| `User_File/Device/Peripheral/Motor/DMmotor/dmmotor.md` | 修正 8 位 CAN ID、位置速度模式语义、已修复的力位混控反向限幅、ServiceAll 仅重试失败入队。 |
| `User_File/Device/Peripheral/Motor/DJImotor/dji_motor.md` | 说明型号与机械减速比的区别、直驱摩擦轮 25 rad/s、反馈缺少一致 Snapshot。 |
| `User_File/Device/Peripheral/Referee/README.md` | 用当前未接入状态、解析限制和阻塞发送契约替换旧 UI 任务教程。 |
| `User_File/Device/Peripheral/Remote/README.md` | 区分 DBUS 与 S.BUS 的拆包、failsafe、超时和当前未接入状态。 |
| `docs/framework_review_2026-09-25.md` | 标为历史快照，补问题现状表；仓库链接改相对路径，参考工程改其 GitHub 仓库 URL。 |
| `CMakeLists.txt`、`User_Config/Board/board_config.h`、`User_File/System/Transport/transport_config.h` | 仅改注释，分清构建、硬件和固定通信配置职责。 |
| `User_File/Application/Chassis/Chassis.cpp` | 注释运动学符号、最短转向及舵向绝对零位前提。 |
| `User_File/Application/Shoot/Shoot.cpp` | 注释直驱 25 rad/s 和事件累加不等于实际发射完成。 |
| `User_File/Application/RobotCmd/RobotCmd.h` | 标明 setter 仅适合 ControlTask 上下文，ISR/UART 回调不可并发写。 |
| `User_File/Device/Peripheral/Motor/DJImotor/dji_motor.h` | 注明 `motor_type` 不决定实际机械传动比。 |
| `User_File/Device/Peripheral/Referee/referee_26.c`、`User_File/Device/Peripheral/Referee/referee_26.h` | 在解析入口和公共接口标出 chunk、短载荷/长度回绕与阻塞发送限制。 |
| `User_File/Device/Peripheral/Remote/remote_control.c`、`User_File/Device/Peripheral/Remote/remote_control.h` | 注明 DBUS 只取本次 chunk 末尾 18 字节及当前未绑定。 |
| `User_File/Middleware/BSP/CAN/bsp_can.h` | 明确软件队列/周期槽的成功返回不等于 HAL、总线或设备完成。 |
| `User_File/Middleware/BSP/UART/bsp_uart.cpp`、`User_File/Middleware/BSP/UART/bsp_uart.h` | 明确 ReceiveToIdle 双缓冲、Ready 指针有效期、chunk 与 parser 分工。 |
| `User_File/System/MessageCenter/topic.h`、`event_queue.h`、`output.h` | 明确 PRIMASK 保护范围、覆盖/溢出、Output 生命周期和未绑定语义。 |
| `User_File/System/Transport/board_transport.h`、`transport_gimbal.cpp`、`transport_chassis.cpp` | 注明固定板型绑定、ISR/任务交接和序号重建边界。 |
| `User_File/Task/Control_Task.cpp`、`Control_Task_Gimbal.cpp`、`Control_Task_Chassis.cpp` | 注明真实任务职责、1 kHz/High1、阻塞点及各板数据流。 |
| `User_File/Task/CanTxTask.cpp`、`StatusTask.cpp`、`BMI088_Task.cpp`、`TransportTask.cpp` | 注明周期、优先级、主要输入输出及不负责的事项；区分 USB 遥测与板间 Transport。 |
| `RoboMaster_Test/README.md`、`Tests/run_all.sh` | 测试清单取实际目录并自动发现全部套件。 |
| `RoboMaster_Test/Tests/Communication/stubs/FreeRTOS.h`、`cmsis_os2.h`、`Tests/Initialization/CMakeLists.txt`、`system_devices.h` | 补齐真实生产单元新增的静态 CAN 队列和 BoardConfig 测试桩，使一键回归可构建。 |

## 删除或更正的过期描述

- Application README 曾把 RobotCmd_Init 写在所有电机应用之后；现按三目标真实顺序列出。
- Referee README 曾声称底盘初始化裁判系统并创建 `refereeUItask`；当前无此接入。
- Remote README 曾把开关映射当作已实现；现标为历史设计示例。
- DM 文档曾限定 `can_id` 为低四位、且称反向时速度限幅会变负；均与当前源码不符。
- 根 README 曾固定为 10 个 suite、57 项测试并链接未跟踪的 `.vscode` 文件；现用实际目录和可追踪入口。
- Gimbal 文档曾把双板 Yaw 总线写成 FDCAN2；GimbalBoard 的两轴实际均在 FDCAN1。

## 验证与边界

- `SingleBoard`、`GimbalBoard`、`ChassisBoard` 固件构建成功。
- `RoboMaster_Test/Tests` 当前有 Boundary、CAN、Chassis、Communication、FilterPolynomial、Fuzzy、Gimbal、Initialization、Output、SBUS、Shoot、Topic、Trajectory、Transport 共 14 个 suite；一键脚本实际运行 64 项，全部通过。文档不固定该数量。
- `RoboMaster_H7` Markdown 相对链接扫描：无本地绝对路径或不存在的相对目标。`git diff --check` 通过。
- 未使用硬件：两板抓包、断线/拥塞、真实 1 kHz 执行时间、功率与温升仍待实测。

## Remaining Issues（代码仍存在，未在本次修复）

1. Referee parser 不跨 DMA 回调拼帧，固定结构复制缺少载荷长度检查，极端长度可能回绕；VTM 解析亦需复核。
2. DBUS 只取 chunk 末尾 18 字节，不支持流式拆包；Remote/Vision 尚未绑定 System_Init 或进入 RobotCmd，输入源仲裁和失联互锁未完成。
3. Transport 无会话标识；序号超时重建后，持续到达的合法重复旧帧可能被当作新会话。无 ACK、重传、独立心跳或事件链路。
4. DJI 公开 feedback 由 ISR 更新，没有整帧一致 Snapshot；RobotCmd setter 也没有并发保护。
5. Shoot 没有摩擦轮就绪、卡弹回退、热量/裁判互锁和完整 FEEDING FSM；成功 Push 不等于物理完成。
6. Chassis 模式枚举尚无完整独立行为；舵轮依赖 `output_total_angle`，需要可靠绝对零位来源及车体方向标定。
7. CAN Bus-Off 尚无完整恢复策略；独立 IWDG、复位原因记录及 System Health/Fault 遥测尚未接入。

本次只修改文档和注释；测试分支另补一键脚本及编译桩，不修改生产行为。

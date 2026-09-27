# 双板底盘命令与反馈链路

## 构建与板级接线

```sh
cmake --preset SingleBoard && cmake --build --preset SingleBoard
cmake --preset GimbalBoard && cmake --build --preset GimbalBoard
cmake --preset ChassisBoard && cmake --build --preset ChassisBoard
```

主机协议测试保存在 `RoboMaster_Test` 分支的 `Tests/Transport`，按仓库的测试分支约定运行。

`Debug` / `Release` 预设仍是原有单板入口；`SingleBoard` 保留原有
`H7_APP_GIMBAL`、`H7_APP_CHASSIS`、`H7_APP_SHOOT` 硬件控制开关，
默认均为 `OFF`。它是安全构建模板；实车须显式开启所需应用，并先核对
电机接线、方向、量程与控制器标定。
双板预设在配置时固定其应用开关，不靠运行时 BoardId 选择应用。
`.ioc` 不再声明默认 FreeRTOS 任务；任务属性和句柄放在用户文件
`board_tasks_common.c`，各目标任务列表放在各自的 `board_tasks_*.c`。
`freertos.c` 仅在 USER CODE 区调用 `Board_CreateTasks()`，可随 CubeMX 重新生成。
板型预设保存在独立的 `CMakeUserPresets.json`，避免生成器重写 `CMakePresets.json` 时丢失。

| 固件 | 本地应用 | 电机 CAN | 板间 CAN |
| --- | --- | --- | --- |
| GimbalBoard | RobotCmd、Gimbal、Shoot | DM: FDCAN1；Shoot: FDCAN3 | FDCAN2 |
| ChassisBoard | Chassis | 车轮: FDCAN1；转向: FDCAN2 | FDCAN3 |

两块板的板间 CAN 收发器应连接到同一条 Classic CAN 总线，并核对双方波特率、终端电阻和共地。`BoardConfig` 管外设接线；`TransportConfig` 管固定消息方向和 CAN ID；CMake 选择应用、设备和任务源码。

## 线上格式

`Gimbal(1) -> Chassis(2)` 的 `ChassisCmd(1)`，标准 ID 为
`(source << 8) | (target << 5) | message = 0x141`。Classic CAN DLC 为 8。

| 字节 | 内容 |
| --- | --- |
| 0 | 8 位序号，逐次发布递增，自然回绕 |
| 1 | 高 4 位协议版本 `1`；低 4 位 `ChassisMode` |
| 2–3 | `vx`，有符号小端 int16，单位 0.001 m/s |
| 4–5 | `vy`，有符号小端 int16，单位 0.001 m/s |
| 6–7 | `wz`，有符号小端 int16，单位 0.001 rad/s |

非法长度、版本、模式、非有限数值及编码范围外的速度会被拒绝。
发送使用 CAN BSP 的最新值周期槽；RobotCmd 每 10 个 1 ms 控制周期刷新一次。
RX 中断只核对 CAN ID、DLC 并复制完整帧和接收时间。控制任务解码和检查序号，
再按**实际接收时间**发布至本地 `Chassis_Command_Topic`。重复或回退序号不会延长命令有效期。
Chassis 每 1 ms 读取这个 Topic，距接收超过 100 ms 就进入 `ZERO_FORCE`。

`Chassis(2) -> Gimbal(1)` 的 `ChassisFeedback(2)` 固定标准 ID 为 `0x222`，
同为 8 字节 Classic CAN 帧。字节 0 是序号；字节 1 的高 4 位是版本 `1`，
低 4 位中 bit 0 为 `enabled`、bit 1 为 `online`，bit 2–3 必须为零；
字节 2–7 分别为 `vx/vy/wz` 有符号小端 int16，单位依次为
`0.001 m/s`、`0.001 m/s`、`0.001 rad/s`。非法 ID、长度、版本、
保留位及编码超量程值会被拒绝。Chassis 每约 10 ms 发布一次本地反馈，
Transport 只在读到新的 Topic 状态时更新 CAN 最新值槽。云台接收中断只复制
帧与接收时间，控制任务校验后按接收时间发布到本地 `Chassis_Feedback_Topic`；
`RobotCmd_GetChassisFeedback()` 只返回 100 ms 内的新鲜 Topic 数据。

Transport 是构建期选定的固定协议适配层，不是 Router 或动态 Topic 路由。
两方向均在有效帧**实际接收时间**距上次接受超过 100 ms 后重建序号基准；
未超时的重复/回退序号、非法帧及延迟处理时已过期的帧不能刷新 Topic 或重建基准。
现有 8 字节协议没有会话标识，因此超时后新到达的合法旧帧无法与重启板首帧区分；
若持续收到合法重复旧帧，超过 100 ms 后也可能重新建立序号基准。这是当前协议边界，
并非已解决的会话防重放机制。
本协议无 ACK、重传、独立心跳或分包。命令和反馈均为 **latest-value** 连续状态；
将来不能丢的跨板事件应使用独立 FIFO，本次没有实现跨板事件传输。
CAN BSP 的离散命令 FIFO 已按 FDCAN1/2/3 分开，一条总线阻塞不影响其他总线，
但每条总线每次 1 ms 轮询最多尝试一帧。

硬件实测需抓取 `0x141` 和 `0x222` 帧确认约 10 ms 刷新；断开板间 CAN
确认底盘超过 100 ms 时效门限后进入 `ZERO_FORCE`；单独重启云台板验证首帧恢复；
注入单总线堵塞验证其他总线不受影响，并量测 1 kHz 控制任务执行时间。

新增通信与队列不使用堆分配，也不创建新任务。`TransportTask.cpp` 实际承担
USB 遥测任务；后续另行改名，本次不改变任务符号。
INS、Gimbal 和 Shoot 板内控制不经过此 Transport。扩展第三块板时新增对应
构建预设、BoardConfig、固定 TransportConfig 绑定以及该消息的编解码和接收发布入口。

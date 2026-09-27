# 双板 ChassisCmd 链路

## 构建与板级接线

```sh
cmake --preset SingleBoard && cmake --build --preset SingleBoard
cmake --preset GimbalBoard && cmake --build --preset GimbalBoard
cmake --preset ChassisBoard && cmake --build --preset ChassisBoard
```

主机协议测试保存在 `RoboMaster_Test` 分支的 `Tests/Transport`，按仓库的测试分支约定运行。

`Debug` / `Release` 预设仍是原有单板入口；`SingleBoard` 保留原有
`H7_APP_GIMBAL`、`H7_APP_CHASSIS`、`H7_APP_SHOOT` 硬件控制开关。
双板预设在配置时固定其应用开关，不靠运行时 BoardId 选择应用。
如果重新生成 CubeMX 的 `Core/Src/freertos.c`，需保留其中由
`Board_CreateTasks()` 取代默认任务创建列表的改动。

| 固件 | 本地应用 | 电机 CAN | 板间 CAN |
| --- | --- | --- | --- |
| GimbalBoard | RobotCmd、Gimbal、Shoot | DM: FDCAN1；Shoot: FDCAN3 | FDCAN2 |
| ChassisBoard | Chassis | 车轮: FDCAN1；转向: FDCAN2 | FDCAN3 |

两块板的板间 CAN 收发器应连接到同一条 Classic CAN 总线，并核对双方波特率、终端电阻和共地。`BoardConfig` 管外设接线；`TransportConfig` 管固定消息方向和 CAN ID；CMake 选择应用、设备和任务源码。

## 线上格式

首版只有 `Gimbal(1) -> Chassis(2)` 的 `ChassisCmd(1)`，标准 ID 为
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

本协议无 ACK、重传、独立心跳或分包。8 位半范围序号能过滤相邻旧帧；如果
发送板重启或链路中断超过 127 次发送周期，恢复时可能需要等待序号进入新的
半范围窗口。期间底盘保持零力矩。硬件实测时应抓取 0x141 帧确认 10 ms 刷新，
断开板间 CAN 确认底盘 100 ms 后禁用输出，并量测 1 kHz 控制任务执行时间。

新框架没有使用堆分配；RTOS 原有任务和 CAN BSP 队列仍沿用工程的内存机制。
INS、Gimbal 和 Shoot 板内控制不经过此 Transport。扩展第三块板时新增对应
构建预设、BoardConfig、固定 TransportConfig 绑定以及该消息的编解码和接收发布入口。

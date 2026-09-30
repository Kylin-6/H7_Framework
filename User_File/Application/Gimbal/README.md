# 双达妙云台

云台由统一 ControlTask 以 1 kHz 调度，两轴均使用现有 `Class_DMMotor` 和 MIT 模式。
QD4310 驱动仍作为独立设备保留，云台不再依赖它。SingleBoard 默认
`H7_APP_GIMBAL=OFF`；GimbalBoard 构建固定启用云台硬件路径。

## 配置与参考来源

在 [Gimbal_Config.h](Gimbal_Config.h) 集中配置；`Gimbal_Init()` 复制默认配置，
也可在启动阶段传入一份 `Struct_Gimbal_Config`。初始化只调用一次，不等待电机、
不自动设置机械零位、不切换控制模式、不写电机持久化参数。电机端须预先设置 MIT 模式。
配置校验失败或驱动注册失败返回 false，`Gimbal_GetStatus()` 返回 CONFIG_ERROR。
电机、PID、目标、Snapshot 和云台状态计时由 `Gimbal.cpp` 的私有 `GimbalContext` 持有；
外部只能通过初始化、周期入口和只读状态接口访问 Application。

| 项目 | 示例 | 来源与限制 |
| --- | --- | --- |
| Yaw / Pitch 总线 | GimbalBoard 均为 FDCAN1；SingleBoard 为 FDCAN2 / FDCAN1 | 由所选 BoardConfig 固定接线，板间链路占用 GimbalBoard 的 FDCAN2 |
| 电机 ID | 1 / 2 | 示例，需与电机端对应 |
| 反馈 Master ID | 0x101 / 0x102 | 示例，不能与同总线现有接收 ID 冲突 |
| 协议量程 | ±12.5 rad、±45 rad/s、±18 N·m | Meta 达妙驱动示例，必须与电机端 PMAX/VMAX/TMAX 相同 |
| Yaw 角度 Kp / 速度上限 | 8 / 8.72664626 rad/s | 参考 basic 云台，速度上限按示例 500 deg/s 转换 |
| Yaw 转矩环 Kp/Ki/Kd | 全部为 0 | 没有可直接移植的达妙整定值，默认无 Yaw 主动转矩 |
| Yaw 转矩 / 积分上限 | 18 / 0 N·m | 转矩上限仅为协议范围示例，须按机构调整；启用 Ki 时同时设置积分限幅 |
| Pitch MIT Kp/Kd | 20 / 1 | 来自 Meta 小米 Pitch 示例，不是已验证的达妙参数 |
| Pitch 位置 / 速度限位 | [-1.5, 0.5] rad / ±1 rad/s | Meta 机构示例，必须按实际机械零位重新标定 |
| Pitch 电机/姿态比例 | 1 | 直接驱动示例；使用正传动比，反向由电机 reverse 配置 |
| IMU 角速度轴 | Yaw Z、Pitch Y，符号均 +1 | 对应本工程 Z-Y-X 姿态定义；安装方向改变时须复核 |

参考文件：basic_framework 的 `application/gimbal/gimbal.c`；Meta-Embedded-NG 的
`application/gimbal/2yaw_gimbal.c`、`application/sentry/sentry_def.h` 和
`module/motor/DMmotor/dmmotor.h`。这是控制结构与数值示例的适配，不代表参考工程已经
实现或验证了本工程的双达妙硬件。许可见 [THIRD_PARTY_NOTICES](../THIRD_PARTY_NOTICES.md)。

## 控制契约

`GimbalCmd` 的角度是 INS 姿态 rad，速度是姿态角速度前馈 rad/s。

- Yaw：最短路径角误差 → 角度比例环 → 叠加速度前馈并限幅 → 速度 PID → 转矩限幅 → `SetTorque()`。
- Pitch：`p_ref = p_motor + ratio * (pitch_ref - pitch_imu)`；
  `v_ref = v_motor + ratio * (pitch_speed_ref - gyro_pitch)`。
  使用一次快照中的电机位置/速度，按配置限幅后发出 MIT 指令，转矩前馈为零。
- 电机反馈及目标均使用经过 reverse 统一的逻辑方向，协议编码只在驱动中翻转一次。
- 姿态轴和选用的机体系角速度须与机构约定匹配；当前不是任意安装姿态的完整坐标变换器。
- LOCK 捕获并保持当前姿态，忽略随后发布的目标字段；IMU 使用新发布的目标。
- 非有限命令、INS、运动反馈或计算结果不会继续驱动闭环；DISABLED 始终优先停机。

## 状态与恢复

`Gimbal_GetStatus()` 返回 DISABLE、ENABLING、READY、FAULT 或 CONFIG_ERROR。
`Gimbal_Init()` 不发送使能；收到活动模式后才启动就绪流程。现有 RobotCmd 启动默认
发布 LOCK，因此打开云台编译选项后会自动进入此流程，不能把示例参数当作上板标定结果。

- INS 必须不超过 10 ms；两轴运动反馈必须小于 100 ms，在线判断不等待 StatusTask。
- DMMotor 在 100 Hz StatusTask 中至少间隔 20 ms 维护使能/失能命令；云台两秒未就绪进入 FAULT，等待一秒后重试。全程不阻塞控制任务。
- 两轴在线且使能、INS 有效持续 100 ms 才进入 READY；中间失能会重新计算稳定时间。
- DISABLED、故障或初始化部分失败时，对已注册电机调用 `RequestEnabled(false)`；
  DMMotor 立即覆盖周期槽为零刚度/阻尼/转矩，并在低频重试失能直至收到新鲜失能反馈。发布失败下一周期继续尝试；停止帧不能
  保证在物理断线时送达，也不会清除已经进入硬件 FIFO 的帧。
- 活动模式下自动恢复。恢复先清空 PID 历史并捕获当前姿态；IMU 等待 READY 后重新
  发布目标，LOCK 直接保持新捕获的姿态，故障前目标不会重放。
- Daemon 只判断反馈活性；DMMotor 根据云台请求维护协议状态，不自动 ClearError。

`GimbalFeedback` 仍为 100 Hz，字段布局不变。`enabled` 表示两轴电机均 ready，
不是软件状态 READY；`ins_valid=false` 时发布零姿态/速度。使能命令提交成功不代表已使能。

本次没有增加命令来源心跳和整车输入仲裁；无新命令时保持最后模式和目标。自动恢复后
需要新目标这一规则，也不能替代上层的遥控失联策略。

## 验证

`RoboMaster_Test` 分支的 `Tests/Gimbal` 编译真实云台、达妙驱动、PID、Daemon
和消息中心，验证协议、控制、失效及恢复。固件构建覆盖默认、仅云台及三应用开启配置。
尚未完成板测：需要验证型号/量程、方向/零位、MIT 增益、Yaw 转矩环、机械限位、
CAN 满载、断线恢复、使能顺序及实际控制周期。

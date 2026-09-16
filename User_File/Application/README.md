# Application layer

本目录按 RoboMaster 常见的 `RobotCmd -> Gimbal / Chassis / Shoot` 边界组织，移植并
适配了 Meta-Embedded-NG `application/` 中可复用的应用层思路。FreeRTOS Task 只负责
调度，设备实例仍由所属 Application 直接控制。

## 当前模块

- `RobotCmd` 是控制命令的唯一发布者。当前没有强行移植工程中不存在的 DT7、VTM、
  视觉、裁判系统和超级电容驱动；后续输入模块通过 `RobotCmd_Set*()` 写入统一命令。
- `Gimbal` 保留本工程已经工作的 QD4310 和 INS 静态 Topic 链路，并增加动态命令、
  低频反馈接口。INS 姿态反馈仍不经过 FreeRTOS Queue。
- `Chassis` 保留 Meta 的四舵轮运动学和最短转向逻辑，电机接口已改为本工程的
  `Class_DJIMotor`。机械尺寸和舵向零位是待标定参数，默认 `CHASSIS=0`。
- `Shoot` 提供摩擦轮、拨盘反转、单发、三发和连发控制，电机接口已改为
  `Class_DJIMotor`。依赖裁判数据和实机阈值的热量/卡弹判断未移植，默认 `SHOOT=0`。

硬件路径通过 CMake 选项 `H7_APP_GIMBAL`、`H7_APP_CHASSIS`、`H7_APP_SHOOT`
分别启用。默认全部为 `OFF`，启用前必须确认 CAN 总线、ID、传动比、机械尺寸、零位和
PID 参数。

## 生命周期

1. `Application_RegisterTopics()` 在 `osKernelInitialize()` 后、调度器启动前注册动态
   Publisher/Subscriber。
2. `Control_Task` 启动后初始化各 Application 拥有的设备。
3. 每个控制周期依次调用 `RobotCmd_Update()`、`Gimbal_Update()`、
   `Chassis_Update()`、`Shoot_Update()`。

动态命令只在内容改变时发布，应用反馈降频到100 Hz。1 kHz INS -> Gimbal 状态链继续
使用静态 `Topic<INS_State>`，不迁入动态 Queue。

## 开源适配说明

参考源为 MIT 许可的 Meta-Embedded-NG。移植保留的是消息契约、应用边界、AGV 运动学
和基础发射控制思路；没有复制其 CMSIS-RTOS v1 `robot_task`，也没有引入 Daemon、
旧 BSP、旧电机框架或当前工程不存在的设备。完整许可见 `THIRD_PARTY_NOTICES.md`。

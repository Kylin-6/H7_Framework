# Remote 遥控接收驱动

本目录有两套不同协议。`Control_Task` 在 `RobotCmd_Init()` 后通过 `Communication_Init()` 把 UART5 绑定为 S.BUS；每周期从完整帧快照更新 `InputState(Remote)`，由 RobotCmd 的固定仲裁使用。DBUS 仍未绑定 UART。

| 驱动 | 协议与 UART | 接收行为 | 在线/失控语义 |
| --- | --- | --- | --- |
| `remote_control.c/.h` | DJI DT7/DR16 DBUS，18 字节；具体 UART 波特率、DMA 接线需按遥控器和 CubeMX 配置核对 | 回调只取本次 chunk 最后 18 字节并校验通道与开关；不跨回调拼帧，拆包或合包中的较早帧会丢失 | 最近 100 ms 有合法帧视为在线；DBUS 代码无独立 failsafe 位解析 |
| `sbus.cpp/.h` | 标准 S.BUS，25 字节；当前 UART5 配为 100000 baud、偶校验、2 停止位、RX DMA（HAL `UART_WORDLENGTH_9B`） | 跨回调累积、查找帧头并在坏字节后重同步，解码 16 路模拟通道及 CH17/18 | 最近 100 ms 收到结构合法完整帧为在线；`SBUS_IsHealthy()` 另要求 frame-lost/failsafe 均未置位 |

两者不可互换输入。BSP 的 ReceiveToIdle 回调只交付 DMA chunk，不保证一回调等于一帧；DBUS 如需实车接入，应先补有界流式解析，并验证断线输出。S.BUS 的 CH1/CH2/CH7/CH10 已映射底盘调试目标；50 ms 帧时效、failsafe 和 200 ms 回中去抖由 Communication 与仲裁检查。它不直接发布 Topic。

旧资料中的拨轮、左右开关与键鼠/视觉模式映射是**历史设计示例**。当前 VTM/键鼠/Vision 只有安全禁用的 InputState 接口，尚未配置实机模式开关和接收协议。

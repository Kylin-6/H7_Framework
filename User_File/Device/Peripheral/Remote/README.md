# Remote 遥控接收驱动

本目录有两套不同协议，均尚未在当前 `System_Init()` 绑定接收回调，也未进入 `RobotCmd` 输入仲裁。初始化 UART BSP 不等于启用遥控协议。未来应由 Device/parser 提供输入状态，再由 ControlTask 上下文的 RobotCmd 统一仲裁。

| 驱动 | 协议与 UART | 接收行为 | 在线/失控语义 |
| --- | --- | --- | --- |
| `remote_control.c/.h` | DJI DT7/DR16 DBUS，18 字节；具体 UART 波特率、DMA 接线需按遥控器和 CubeMX 配置核对 | 回调只取本次 chunk 最后 18 字节并校验通道与开关；不跨回调拼帧，拆包或合包中的较早帧会丢失 | 最近 100 ms 有合法帧视为在线；DBUS 代码无独立 failsafe 位解析 |
| `sbus.cpp/.h` | 标准 S.BUS，25 字节；当前 UART5 配为 100000 baud、偶校验、2 停止位、RX DMA（HAL `UART_WORDLENGTH_9B`） | 跨回调累积、查找帧头并在坏字节后重同步，解码 16 路模拟通道及 CH17/18 | 最近 100 ms 收到结构合法完整帧为在线；`SBUS_IsHealthy()` 另要求 frame-lost/failsafe 均未置位 |

两者不可互换输入。BSP 的 ReceiveToIdle 回调只交付 DMA chunk，不保证一回调等于一帧；DBUS 如需实车接入，应先补有界流式解析，并验证断线输出。S.BUS 虽具备拼帧和 failsafe 检查，仍没有命令映射、死区、极性或急停策略，也不会自动发布 Topic。

旧资料中的拨轮、左右开关与键鼠/视觉模式映射是**历史设计示例**，当前代码没有实现这些模式。不能据此判断实车安全行为。

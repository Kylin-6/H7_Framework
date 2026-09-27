# 裁判系统协议模块

## 当前状态

本目录保留裁判系统帧解析、UI 绘制与 VTM 解析代码。当前 `System_Init()` 只初始化 UART BSP，没有调用 `RefereeInit()`；`Control_Task` 和板级任务列表也没有裁判系统 UI 任务。底盘应用没有消费裁判数据，功率、热量和发射互锁尚未接入。因此不能把本模块当作已运行的比赛功能。

## 协议解析与 UART 接入

调用 `RefereeInit(UART_HandleTypeDef*)` 才会注册 `RefereeRxCallback()`；传入的 UART 需已具备 ReceiveToIdle RX DMA。BSP 回调参数是一段 DMA chunk，可能包含半帧、多帧或杂字节。`JudgeReadData()` 用 320 字节静态缓存跨 chunk 重组帧，验证帧头 CRC8 和整帧 CRC16，然后按 CmdID 复制到 `referee_info_t`。`RefereeReceiveData()` 也调用相同解析器。VTM 使用本目录的独立解析入口。

已知固定长度命令要求声明长度、协议常量和目标结构大小完全一致；超长帧被拒绝，CRC 错帧逐字节重同步。解析状态由 UART 回调持有；`RefereeReceiveData()` 不应与 UART 中断并发调用。

## 在线状态与发送

`RefereeIsOnline()` 以最近一次 CRC 校验通过的完整帧为依据，超时门限为 500 ms；它不保证每个业务字段都已更新或有效。`RefereeSend()` 在 UART 提交成功后调用 `osDelay(115)`，只能在允许阻塞的任务上下文调用，不能从 ISR 或 1 kHz 控制周期调用。调用它之前必须先完成 `RefereeInit()`。

## Known Issues

- `referee_info_t` 仍由中断更新，任务直接读取多个字段可能跨帧；消费前需增加业务快照。
- 数据尚未进入本地 Topic、Chassis 或 Shoot；没有当前固件的 `refereeUItask`。
- UI 发送的 115 ms 延时需要单独的低优先级调度和实机速率验证。

## 历史 UI 示例

旧框架使用 `MyUIInit()`、`MyUIRefresh()`、`UICharDraw()`、`UIRectangleDraw()`、`UIFloatDraw()`、`UILineDraw()` 与 `UIRefresh()` 绘制功率条，并在独立 UI 任务以约 10 Hz 刷新。它仅是移植参考；当前板级任务没有创建该任务，示例中的旧类型和数据源不可直接复制到生产代码。

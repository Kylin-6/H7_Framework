# 裁判客户端 HUD 示例

将 Infantry_Chassis HUD 的布局和非阻塞刷新移入框架，使用现有裁判常规链路
`0x0301` 绘图。示例只读取本板 Topic，不发布控制命令。

## 启用与接线

`H7_APP_CLIENT_UI` 默认 OFF，三个板型均可显式开启；使用当前所选 preset，例如：

```sh
cmake --preset SingleBoard -DH7_APP_CLIENT_UI=ON
cmake --build --preset SingleBoard
```

这个选项独立于 Gimbal/Chassis/Shoot 开关，不自动开启机构控制。
`BoardHardware::referee_uart` 默认绑定 USART10：PE2 RX、PE3 TX，115200 8N1，
已有 RX/TX DMA。由 `System_Init()` 一次性调用 `RefereeInit()`，不先重复启动同一串口。
开启 UI 后此串口由 Referee 独占，不再绑定其他接收器或发送器。
配置缺少 RX/TX DMA 或 Referee 注册失败时，`ClientUI_Init()` 返回 false，只停用 UI。

正常接线：PE3/TX → 裁判电源模块 User RX，模块 User TX → PE2/RX，并共地。
默认 `ClientUI_Config.h` 的 `kRobotId=0`，只在收到合法、新鲜的 `0x0201` 后发送。
支持红/蓝步兵 3/4/5；客户端接收 ID 为机器人 ID + `0x100`。
身份年龄达到 500 ms 后暂停发送；恢复或身份改变后重新初始化本 UI 图层。
固定身份调试时将 `kRobotId` 显式设为 3/4/5/103/104/105，仍使用已有 DMA 配置，
可只接 PE3/TX 和共地。裁判主控身份与登录客户端必须一致。

## 分层与调用

- Device 的 [非阻塞发送接口](../../Device/Peripheral/Referee/referee_ui_sender.h) 负责图元、文字、图层删除的组包、CRC 和 UART 提交，返回值只表示软件提交结果。
- `ClientUI_Capture.cpp` 是示例数据适配器，ControlTask 在机构更新后调用，按统一时间源每 10 ms 最多采集一次。
- `Client_UI_State_Topic` 是唯一显示快照，ControlTask 写入，StatusTask 读取。
- `ClientUI.cpp` 管刷新步骤、可见性、限频和重建；`ClientUI_Layout.h` 只根据显示值生成图元。
- StatusTask 启动时调用 `ClientUI_Init()`，每 10 ms 在设备安全服务之后调用 `ClientUI_Update()`；每次最多提交一包，无延时、动态分配或新任务。

具体机器人可替换 Capture 的字段映射，在同一 ControlTask 发布显示快照。
不要另建同用途 Topic 或增加并发发布者。刷新状态仅由 StatusTask 持有；
`ClientUI_Init(robot_id)` 的参数允许在显式初始化时覆盖默认身份，不是并发运行期 setter。

## 显示及数据时效

采用 1920×1080、左下角原点、第 9 层和固定 `HU` 图元名称，避免其他 UI 使用同层同名。

| 区域 | 数据与行为 |
| --- | --- |
| 中央准星、参考线 | 保留原 HUD；未经弹道、距离或相机偏心标定 |
| Pitch 弧形指针 | `Gimbal_Feedback_Topic` 新鲜且 `ins_valid` 的 Pitch，内部 rad，显示夹至 ±40°；无效则删除 |
| 底盘状态圆环 | 新鲜 Chassis 命令与反馈；未知/离线/等待为青色，停机/正常为粉色，enabled 且 ROTATE 请求为绿色，不证明实际旋转 |
| 圆环缺口 | 示例缺少相对 Yaw，显示完整圆环；不使用 INS Yaw 冒充相对角 |
| 双柱 | 新鲜、online 的 ShootFeedback 左右绝对转速，显示换算 kRPM，0–7000 RPM，超范围夹顶；无效则删除 |
| LOAD | 未知白色、反馈绝对速度 ≤1 rad/s 粉色、>1 rad/s 绿色；不表示弹丸已发出 |
| 模式文字 | STOP / FREE / FOLLOW / SPIN；数据不足为 UNKNOWN，不带老车档位 |
| A/B/C | 保留静态位置；示例缺少可靠自瞄采用状态，亮条隐藏 |
| 左侧电容表 | 保留位置，标注 CAP N/A，不用裁判缓冲能量替代电容电量 |
| 三行报警 | 默认空；具体机器人可提交每行至多 29 字节 ASCII，黄色提示/橙色故障，不包含老车诊断位或去抖策略 |

反馈有效期为 100 ms（恰好门限仍有效）。快照保留 Pitch、Shoot、Chassis 的原始源时间戳，
Capture 不会给旧反馈续期；StatusTask 再次检查源时间，采集停止时也不延长反馈有效期。
整体显示快照超过 200 ms 后，所有动态字段降为未知，并清除旧报警。
非有限角度/转速不绘制；缺失的跨板姿态和发射数据不会自动转发。

## 刷新与恢复

相邻成功提交间隔至少 100 ms；稳定时动态图元约 10 Hz，存在辅助包时交替发送，
动态约 5 Hz。辅助包优先级为报警变化、模式文字、静态重建。
首次仅删除第 9 层，然后交替刷新七个动态图元、37 个静态图元和六个静态标签。
无变化报警且 UART 可提交时，首次完整布局约 2.7 s。
每 5 s 开始重新 ADD 静态图元、标签、动态图元及辅助文字，修复客户端重启或丢图，
不周期清屏；这里的 5 s 是开始重建的间隔，不保证 5 s 内全部完成。

首次显示用 ADD，正常更新用 Change；无效图元用 Delete，恢复后重新 ADD。
DMA 忙或提交失败时不推进绘制步骤、发送序号和成功提交时间，下次轮询重试。
客户端没有逐包确认，成功提交不表示客户端收到；停发后已画出的客户端图形可能保留。

## 客户端面板遮挡

示例沿用原 HUD 坐标，尚未针对客户端自带“射击初速度 / 允许发弹量”等浮动面板避让。
这类面板可能挡住右侧 Pitch 刻度与模式文字；提高发送频率或改变图元层号不能保证解决。
部署时需要按实际客户端布局移动面板或调整 HUD 坐标，并检查最长模式/报警文字的范围。

## 验证

主机替身运行真实解析器、CRC、Topic、Capture、布局与刷新实现：

```sh
python3 tests/client_ui/run.py
```

需要主机 GCC/G++，产物仅写入 `build/tests/client_ui/`。默认开启 AddressSanitizer 和
UndefinedBehaviorSanitizer；受 ptrace 限制的环境可用
`ASAN_OPTIONS=detect_leaks=0 python3 tests/client_ui/run.py`，保留地址和未定义行为检查。
测试覆盖组包/CRC/ID、坏帧和半帧、500 ms 身份过期/恢复、DMA Busy/失败重试、
100 ms 限频、5 s 重建、ADD/Change/Delete、数据失效、非有限值、Pitch/转速限幅和序号回绕。

实机仍需验证 UART/客户端渲染、任务栈水位、Pitch 方向、客户端面板避让、
重启后的重建，以及源数据中断后的隐藏/未知显示。主机通过不能代替实机验收。

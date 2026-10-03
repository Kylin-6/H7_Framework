# Input 输入适配与仲裁指南

Input 将设备解析结果转换为 SI 命令，保存固定来源状态，再由 SourceArbitration 选择本周期目标。
RobotCmd 根据仲裁结果发布命令；Input 不直接控制电机，不另建消息总线。
当前仅绑定 UART5 S.BUS，且只映射底盘；VTM、Keyboard、Vision 有提交接口，尚未绑定实际设备。

## 文件与职责

| 文件 | 负责什么 | 移植时处理什么 |
| --- | --- | --- |
| [remote_input.h](remote_input.h) / [remote_input.cpp](remote_input.cpp) | S.BUS 初始化、帧健康判断、回中解锁和底盘通道映射 | 接收器接口、通道、方向、死区、速度档和恢复条件 |
| [input_state.h](input_state.h) / [input_state.cpp](input_state.cpp) | 固定来源数据、选择状态和统一时间 | 新输入提交位置、完整命令、有效性与接收时戳 |
| [source_arbitration.h](source_arbitration.h) / [source_arbitration.cpp](source_arbitration.cpp) | Remote 许可、选源、时效、命令合法性和 Vision 覆盖 | 来源策略、门限和新字段校验 |

协议解析归 [S.BUS Device](../../Device/Peripheral/Remote/sbus.h) 与 BSP。
命令发布归 [RobotCmd](../RobotCmd/README.md)，机构算法归 Gimbal/Chassis/Shoot。
整体调度见 [Application 指南](../README.md)。

## 任务顺序和并发约定

启动顺序为 RobotCmd_Init → RemoteInput_Init → 机构初始化。
RemoteInput_Init 清空 InputState，设置本地时间并调用 SBUS_Init(&huart5)；初始化失败保持未解锁。

```text
UART/DMA → S.BUS Device 完整帧快照
  → ControlTask：RemoteInput_Update（设置本周期时间、更新 Remote）
  → 新输入适配 Update（需要时提交 VTM / Keyboard / Vision）
  → RobotCmd_Update：InputState_Read → SourceArbitration_Resolve
  → 各机构 Update
```

新增输入适配应放在 RemoteInput_Update 后、RobotCmd_Update 前，当前任务没有这些额外调用。
InputState 是一个静态结构体，没有锁或临界区；所有读写和选择接口必须在同一 ControlTask 上下文调用。
不要从 ISR 或另一个任务直接 Submit。异步解析器应先提供同步完整快照，再由 ControlTask 转换和提交。

`now_ms` 与 `received_ms` 使用本地 HAL_GetTick 毫秒时基；不能把协议中的远端未同步时间直接写入。
接收时戳应保留实际新帧到达时间，不要在每次 Update 时给旧帧重新盖时间戳。
若替换 RemoteInput，仍需每周期调用 InputState_SetTime，避免仲裁时钟停在旧值。

## 当前 S.BUS 映射

通道数组为零基索引，代码中的 index 0 对应遥控 CH1。

| 通道 | 索引 | 当前用途 |
| --- | --- | --- |
| CH2 | 1 | X 平移，正号映射 |
| CH1 | 0 | Y 平移，映射时取负号 |
| CH7 | 6 | 速度档，映射到 0～1 并缩放平移/旋转目标 |
| CH10 | 9 | 只有负半轴参与手动旋转，正半轴不生成旋转目标 |
| CH1～CH4 | 0～3 | 解锁回中检查，不表示这四个通道都已映射控制 |

Axis 将中心化通道值除以 784，裁剪到 [-1,1]，在归一化值 (-0.04,0.04) 内置零。
平移最大绝对值为 0.5 m/s，旋转为 1 rad/s，由 input_state.h 的常量限定。
任一底盘速度非零时使用 NO_FOLLOW，否则保持默认 ZERO_FORCE。
CH5 跟随模式、云台角度和发射按钮当前均未接入；提交的 Remote 云台默认为 DISABLED，Shoot 默认为 OFF。
修改映射时同步核对车体坐标、遥控方向和 Application 对模式的实际支持。

### 帧健康与解锁

健康条件为接收器初始化成功、取得完整帧、帧年龄不超过 50 ms，且 frame_lost/failsafe 均未置位。
不健康时立即取消 armed、重置恢复计时并提交无效 Remote 状态。
尚未解锁时 CH1～CH4 必须在 ±50 原始单位内，健康且回中持续 200 ms 后才提交有效命令。
未回中或再次失联都会重新计时；已解锁后不再要求持续回中。
CH10 不参与解锁回中检查，因为当前旧遥控上它是偏置开关。

## InputState 接口

| 接口 | 行为与使用时机 |
| --- | --- |
| InputState_Reset | 清空四份输入与选择状态，默认选择 Remote、禁用 Vision；启动时调用 |
| InputState_SetTime | 设置本周期仲裁时间，单位 ms |
| InputState_SubmitRemote | 复制完整 Remote 命令和有效性；没有附加校验 |
| InputState_SubmitVtm | 复制完整 VTM 状态；提交不会自动切换来源 |
| InputState_SubmitKeyboard | 复制完整键鼠状态；提交不会自动切换来源 |
| InputState_SubmitVision | 复制绝对 INS 瞄准角；不提供底盘或发射目标 |
| InputState_Select | 接受 Remote/Vtm/Keyboard，非法枚举被忽略；控制来源或 Vision 开关变化会更新 selected_at_ms |
| InputState_Read | 按值返回当前完整结构，不消费、不进行仲裁或时效校验 |

ControlInput 同时包含 chassis、gimbal、shoot、received_ms 和 valid，提交是整体覆盖，不是字段合并。
即使只接入一种机构，也要为其他命令明确提供安全默认值。
VisionAimInput 的 yaw/pitch 为 INS 坐标系绝对 rad，不是像素偏移或相对角速度。
原始 degree、像素或摇杆量应在输入适配边界完成单位/坐标转换。

## 来源仲裁规则

SourceArbitration_Resolve 是纯决策入口，不修改 InputState、不发布 Topic、不访问设备。
默认结果 armed=false，机构命令均为安全默认值。

1. Remote 必须 valid、年龄不超过 50 ms 且命令合法；VTM/键鼠不能绕过 Remote 安全许可。
2. selected 只能是 Remote/Vtm/Keyboard，读取被选中的完整 ControlInput。
3. 选中 Remote 的时效为 50 ms，VTM/Keyboard 为 100 ms；接收时戳不得早于 selected_at_ms。
4. 选中输入失效时保持安全结果，不自动退回其他来源。
5. 条件通过后复制三类命令并设置 armed=true，再处理可选 Vision 云台覆盖。

CommandValid 校验所有命令枚举及数值有限性，底盘平移/旋转还检查上述 0.5 m/s、1 rad/s 边界。
当前没有为云台角目标/速度或 Shoot 速度/射速设置额外数值上限；机械限位与机构控制约束仍需由应用处理。
扩展消息字段或枚举时须同步校验，否则新字段可能绕过边界或新模式被拒绝。

Fresh 用 uint32_t 毫秒差判断年龄，门限相等仍有效；AtOrAfter 用无符号差判断切源先后，
比较跨度须小于半个 uint32_t 计数周期。两个时戳必须处于同一本地时基。
来源或 Vision 开关变化都会建立新的时间边界，需要收到不早于该时刻的选中输入。
反复选择相同来源和相同 Vision 开关不会重置边界。

### Vision 覆盖

仅 vision_enabled=true 且选中命令的云台未 DISABLED 时处理视觉目标。
Vision 必须 valid、年龄不超过 100 ms、不早于切源时刻，且两轴角度有限。
通过后云台切 IMU 并使用视觉绝对角，速度前馈清零；失败时云台切 LOCK，其他机构仍使用选中来源目标。
开启 Vision 不会解锁失效 Remote，也不会给已 DISABLED 的云台自动授予控制许可。

## 接入代码例程

以下函数由新增适配器在 ControlTask 调用；协议解析和异步快照由对应 Device 实现。
示例只展示应用接口，不假设工程已有键鼠或视觉驱动。

<details>
<summary>提交键鼠底盘目标：完整命令、时间戳和失效处理</summary>

```cpp
#include "input_state.h"

void SubmitKeyboard(bool packet_valid, float forward_m_s, uint32_t received_ms)
{
    ControlInput input; // 其他机构保留默认安全命令。
    if (packet_valid) // 解析器确认取得合法的新鲜来源快照，再交给仲裁检查时间与数值范围。
    {
        input.chassis.mode = ChassisMode::NO_FOLLOW;
        input.chassis.velocity_x_m_s = forward_m_s; // 必须为 m/s，示例范围 ±0.5。
        input.received_ms = received_ms; // 使用实际接收时刻，不能每周期替旧包刷新。
        input.valid = true;
    }
    InputState_SubmitKeyboard(input); // 无效时提交默认 valid=false，仲裁不会沿用有效旧状态。
}
```

</details>

<details>
<summary>显式选择键鼠来源：放在遥控模式切换边沿</summary>

```cpp
void SelectKeyboard(uint32_t now_ms)
{
    InputState_Select(InputSource::Keyboard, false, now_ms); // 关闭视觉覆盖，建立来源切换边界。
    // 后续键鼠数据接收时戳须不早于 now_ms，Remote 仍必须健康并通过回中解锁。
}
```

</details>

<details>
<summary>提交 Vision 绝对角：不直接发布云台命令</summary>

```cpp
void SubmitVision(bool packet_valid, float yaw_rad, float pitch_rad, uint32_t received_ms)
{
    VisionAimInput aim;
    if (packet_valid) // 协议已解析为 INS 坐标绝对 rad；仲裁再检查有限性、年龄和切源边界。
    {
        aim.yaw_angle_rad = yaw_rad;
        aim.pitch_angle_rad = pitch_rad;
        aim.received_ms = received_ms;
        aim.valid = true;
    }
    InputState_SubmitVision(aim);
}
```

开启视觉时由显式模式切换调用 InputState_Select(当前来源, true, now_ms)。
选中来源必须先提供非 DISABLED 的云台命令；当前 Remote 默认禁用云台，需要先补充遥控映射。

</details>

## 重新开发的修改位置与验证

增加云台/发射遥控映射时，在 RemoteInput_Update 构造的 ControlInput 中填写相应命令。
连续 Shoot 状态写入 shoot，单发按钮边沿使用 RobotCmd 的事件流程；不能用每周期 ON 代替单发事件。
事件提交需与本周期仲裁许可、切源清队列顺序协调，具体见 [RobotCmd](../RobotCmd/README.md)
和 [Shoot](../Shoot/README.md)。

新增协议输入时保留“Device 解析 → 同步快照 → 输入适配转换 → Submit → 仲裁”的分层。
新增源文件和 include 在根 CMake 显式注册，检查 SingleBoard/GimbalBoard 的源码选择。
InputState_Reset 会清空全部来源，不应在某个来源临时失联时反复调用，失联只提交该来源无效状态。

验证应覆盖初始化失败、S.BUS 帧丢失/failsafe、50 ms 门限、200 ms 回中解锁、
通道方向/死区/速度档、切源前旧包被拒、选中来源失效不回退、Vision 失效 LOCK，
以及 Remote 失效时三个机构最终收到安全命令。
纯文档修改核对内容和链接；输入行为变更还需构建 SingleBoard/GimbalBoard，并实测遥控失联与恢复。

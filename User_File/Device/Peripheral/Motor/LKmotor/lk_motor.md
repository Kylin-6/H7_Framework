# LK 电机驱动

本驱动面向瓴控 **MG4005-i10v3**，使用当前框架的 BSP CAN、Daemon 和 Class_PID，提供
单机电流指令、四电机广播电流指令及主控串级 PID。未创建实际设备实例，未修改机构控制。
驱动已接入所有板型的源码列表及现有 100 Hz StatusTask；无实例时服务直接返回。

## 资料依据与待实测参数

协议以用户提供的厂家资料为准：

| 文件名 | 文档及依据 |
| --- | --- |
| `20260326142653f.pdf` | 《电机 CAN 总线通讯协议》V2.36，第 5～10 页：状态、启停、单机电流命令 |
| `20260317100100f.pdf` | 《电机 CAN 广播通讯协议》V2.35，第 1～3 页：0x280、0x288、节点与频率限制 |
| `20260609135629f.pdf` | 《MG40～50 电机驱动系统使用说明》，第 4、10、14、15 页：型号、广播设置、轴侧与显示单位 |

参考 `basic_framework/modules/motor/LKmotor` 的电流控制和多圈展开思路，按本框架重新实现。
不沿用参考代码的动态分配、全局第一台电机发送对象或 PID 误接线。

资料存在以下差异，第一版明确保留边界：

- V2.36 将 MG 转矩电流定义为 `66/4096 A/LSB`，MG 使用说明的显示表却给出其一半。
  控制、反馈、电流环及电流前馈统一使用 **raw**，不公开未经确认的 A/N·m 换算。
- 单机协议允许 ±2048，广播协议与 MG 使用说明使用 ±2000；两种模式统一采用 **±2000 raw**。
  `current_limit_raw` 在此范围内进一步限幅，默认 **0**。这不是电机额定或峰值电流。
  A1 不受上位机 Max Torque Current 限制，不能把上位机限幅当作本接口保护。
- MG 硬件标称 18 位编码器，运动回包只给两字节，未说明本型号映射。
  `encoder_counts` 暂定 **65536**，可配置协议列出的 16384、32768、65536；这表示
  **CAN 反馈每转子圈计数**，不是物理编码器分辨率，不能填入 262144。
- `gear_ratio` 默认 **10.0**。反馈的转子速度按 `1 deg/s/LSB` 转 rad/s，再除减速比；
  单圈与累计角同样先转换到转子 rad，再除减速比。暂按编码器递增与正速度同向解释。
- 广播协议新增速度/位置命令，但 MG 使用说明仍写仅支持扭矩广播；第一版只实现广播电流控制。

**以上编码器映射和正方向尚未实机确认。** 接入时对照上位机与 CAN 原始反馈，缓慢转动
输出轴一圈核对计数和回绕次数，并核对固件、电流比例及温度符号。参数由应用配置传入，
修改配置后重新初始化固件；不提供运行期切换总线模式、减速比或编码器范围的接口。

## 初始化和所有权

使用 `Struct_LKMotor_Init_Config` 配置 `Class_LKMotor::Init()`，对象和外部反馈/前馈变量
必须具有足够长的生命周期，推荐静态对象。构造不访问硬件；在时间戳、CAN BSP 已准备好后初始化，
完成全部电机和 Group 注册后再开始控制，不能在控制运行期间追加设备或移动/复制对象。

初始化检查总线、ID、量程、有限数值、PID 周期及环配置。`Init()` 失败后不得使用该对象控制；
一旦进入 BSP/Daemon 注册阶段，失败对象也须保留静态生命周期且不得重新 Init，因为框架没有注销接口。
Daemon 的框架总容量仍为 32 个设备，不因 CAN 节点范围为 32 就保证可同时注册 32 台 LK。

- 单机模式 `SINGLE`：节点 1～32，收发 ID 均为 `0x140 + can_id`。
- 广播模式 `BROADCAST`：节点 1～4，电流发送 ID 为 `0x280`，各自回复仍为 `0x140 + can_id`。
- 同一总线的 LK 实例必须使用相同模式，不同总线可采用不同模式。
- 通过厂家上位机预设并保存广播模式、波特率和 ID，驱动不写 ROM、不自动切换电机内部配置。
- 同一总线只允许一个广播 Group，必须包括该总线的全部已注册 LK 实例；组内参数顺序决定目标顺序，
  CAN 字节位置始终由节点 ID 决定。允许缺少物理 ID，例如只使用 1、3，但不允许 Init 参数中间留空。
  Group 创建后禁止再注册同总线电机，未成组的广播实例始终不能 ready。

电流周期帧走 `CAN_Tx_Perform()`；启停、清错和查询走 `CAN_Tx_Submit()`。
广播离散命令使用 `0x288`，只填写对应节点的命令字，其他字节为零。
**所有发送返回值仅表示软件接受，不表示 HAL 已发送或电机已执行。**

## 单机使用

以下例程保留零输出默认值；电流限幅和 PID 参数由机构实际参数传入，不将示例参数视为可上机值。

```cpp
#include "lk_motor.h"

static Class_LKMotor motor;

bool Example_Init(FDCAN_HandleTypeDef *bus, float checked_current_limit_raw,
                  const PID_InitTypeDef &speed_pid)
{
    Struct_LKMotor_Init_Config config{};
    config.hfdcan = bus;
    config.can_id = 1;
    config.mode = Enum_LKMotor_Mode::SINGLE;
    config.gear_ratio = 10.0f;
    config.encoder_counts = 65536; // 临时值，实测后修改。
    config.current_limit_raw = checked_current_limit_raw;
    config.close_loop = LK_MOTOR_SPEED_LOOP;
    config.outer_loop = LK_MOTOR_SPEED_LOOP;
    config.speed_pid = speed_pid; // D_T 必须等于调用 Control 的周期。
    return motor.Init(config);
}

// 仅在 Init 成功后由唯一控制任务周期调用。
bool Example_Update(bool allowed, float speed_rad_s)
{
    const bool requested = motor.RequestEnabled(allowed);
    const auto snapshot = motor.GetFeedbackSnapshot();
    // 必须先请求使能；ready 用于应用的姿态捕获/恢复，不能反过来作为请求使能的前提。
    if (!requested || !snapshot.ready) { return false; }
    motor.SetRef(speed_rad_s);
    const bool calculated = motor.Control();
    const bool submitted = motor.Send(); // Control 失败也发送安全零目标。
    return calculated && submitted;
}
```

`SetRef()` 按 outer_loop 使用输出轴累计 rad、输出轴 rad/s 或电流 raw。
`Control()` 只计算，`Send()` 才发布；BSP 只发送更新过的周期槽，调用方需按实际控制周期持续更新。
`Send()` 未就绪时仍可成功提交零电流，因此判断设备可用性应读取快照的 `ready`。

主控环按角度→速度→电流顺序执行：角度环只在 angle 为外环时运行，速度环在 angle/speed
为入口且已启用时运行，电流环只要已配置就运行。开环目标为 raw；若只启用位置环而没有速度环，
位置 PID 输出须直接按电流 raw 配置。`speed_feedforward` 仅在执行速度环时加到速度目标；
`current_feedforward` 加到电流环目标，未启用电流环时加到最终指令。

角度、速度可分别选择 `EXTERNAL`，指针单位与正方向由调用方统一。外部变量由应用保证同步，
不能指向临时局部变量。非有限目标、参与计算的外部反馈或前馈会清零输出、返回 false，并要求
下次有效控制前重置 PID 历史。`reverse` 反转内置运动反馈和最终输出，内部电流环也相应转换测量符号；
原始 `encoder/current_raw` 始终保留报文内容。PID 对象由设备内部持有，配置在 Init 后固定。

## 广播组使用

```cpp
static Class_LKMotor motor1, motor3;
static Class_LKMotor_Group motors;

bool GroupExample_Init(FDCAN_HandleTypeDef *bus, float checked_current_limit_raw)
{
    Struct_LKMotor_Init_Config config{};
    config.hfdcan = bus;
    config.mode = Enum_LKMotor_Mode::BROADCAST;
    config.current_limit_raw = checked_current_limit_raw;
    config.can_id = 1;
    if (!motor1.Init(config)) { return false; }
    config.can_id = 3;
    if (!motor3.Init(config)) { return false; }
    return motors.Init(&motor1, &motor3);
}

bool GroupExample_Update(bool allowed, float current1_raw, float current3_raw)
{
    const bool requested = motors.RequestEnabled(allowed);
    const bool controlled = motors.Control(current1_raw, current3_raw);
    return requested && controlled;
}
```

组 `Control(ref...)` 设置目标、计算所有成员并提交整帧；即使一台未就绪，也发送其他成员目标，
但返回 false。亦可分开 `SetRef()`、无参数 `Control()`、`Send()`。
广播电机的单机 `Send()` 返回 false，常规发送权归 Group；设备服务和失能请求可覆盖安全组帧。
未注册槽保持零，安全覆盖只清除不安全成员，不能覆盖仍就绪成员的目标。

广播需 500 kbit/s 或 1 Mbit/s。厂家给出四电机命令上限约 600 Hz / 1.2 kHz；
这些上限还未扣除本驱动状态查询、启停和同总线其他设备的流量。接入时应计算总线预算，
500 kbit/s 不应直接以 1 kHz 运行四电机广播。

## 状态、安全服务与恢复

`GetFeedbackSnapshot()` 在短临界区内返回运动、状态及各自的时间戳。

| 字段 | 语义 |
| --- | --- |
| `online` | 收到合法 A1/9C 运动回复且未超过 `feedback_timeout_ms`，默认 50 ms |
| `status_valid` | 合法 9A/9B 状态回复未超过 `status_timeout_ms`，默认 100 ms |
| `requested_enabled` | Application 的期望许可，掉线不会擅自更改 |
| `actual_enabled` | 新鲜状态报告 `motor_state == 0x00` |
| `fault` | 新鲜状态中的 error_state 非零；过期不能解释为无故障可用 |
| `ready` | 初始化/所有权有效、请求使能、运动及状态新鲜、无故障，并收到开启提交之后的开启状态 |

运动回复才 Feed Daemon；状态、启停确认或未知命令不延长运动在线时间。ISR 只校验、解析和登记
安全标志，不发送命令、不计算 PID。状态表只接受 `0x00` 开启和 `0x10` 关闭，其余值不刷新状态时间。

初始化登记安全关闭要求。第一次 `RequestEnabled(true)` 先发布零目标并提交必要的关闭操作；
收到关闭状态和运动反馈后，ServiceAll 提交开启，再等待开启之后的新状态回复。
这使启动和掉线恢复共用相同的确认顺序，应用无需先等待 ready 或手动查询。
使能回显 0x88 本身不构成 ready；重复同一个请求不重复提交命令，待提交失败仍返回 false。

`RequestEnabled(false)` 首次调用及 true→false 边沿立即清零并尝试关闭；安全周期槽与离散命令
分别检查返回值。ServiceAll 每 10 ms 检查安全条件并补交失败操作，每 20 ms 查询状态，
未就绪时同时查询运动反馈；未确认的启停命令按不快于 20 ms 的节奏重试，软件提交失败在下次服务重试。
`ClearError()` 仅显式调用时提交，失败由服务重试；设备仍有故障时不会自行反复清错。

失能、运动超时、状态过期或故障禁止非零指令。Control/Send 即时检查，应用不再调用时由服务在
下一次调度覆盖零目标并维护关闭命令；从反馈/超时发生到服务处理最多还有一个 10 ms 调度周期，
之后仍受 CAN 队列、硬件 FIFO 和总线时延影响。已经进入发送通道的离散命令不能撤回，驱动维护
最新期望状态并以反馈确认，不把软件提交当作设备执行。

掉线恢复先确认关闭，再按保留的应用许可开启；故障必须由上层显式处理。没有新 Control 计算时，
恢复后的 Send 仍保持零。PID 历史由控制任务在下次有效计算前重置，服务和 ISR 不并发修改算法对象。
Control 计算期间发生安全状态变化时，版本检查会丢弃旧计算结果。

编码器首帧以当前单圈计数建立累计角；正常反馈使用半圈阈值展开，相邻反馈期间转动必须少于半转子圈。
运动超时后首帧重新建立基准，不补算失联圈数，不承诺跨掉线/掉电累计位置连续。应用根据 ready
恢复边沿重新捕获姿态与目标。配置错误和跨多圈失联无法靠软件展开恢复真实绝对角度。

协议没有事务序号；启停确认只保证在软件提交后观察到新鲜状态，不能严格关联 CAN 队列中迟到的回复。
不提供 ROM 置零、参数烧写、电机原生速度/位置环或自动配置广播模式。

## 验证

固件验证沿用当前选中的 ChassisBoard / Debug：`cmake --build build/ChassisBoard`。
编译不能替代实机接线、方向、编码器映射、电流比例、总线负载及拔线测试。

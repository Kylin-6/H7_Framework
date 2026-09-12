# DJI 电机驱动

`Class_DJIMotor` 支持 M2006/C610、M3508/C620 和 GM6020，使用项目当前的
FDCAN 回调注册和周期发送接口，不使用动态内存。

## 协议配置

| 设备 | ID | 反馈 ID | 控制 ID | 指令范围 |
|---|---:|---:|---:|---:|
| M2006/C610 | 1~4 | `0x200 + ID` | `0x200` | ±10000 |
| M2006/C610 | 5~8 | `0x200 + ID` | `0x1FF` | ±10000 |
| M3508/C620 | 1~4 | `0x200 + ID` | `0x200` | ±16384 |
| M3508/C620 | 5~8 | `0x200 + ID` | `0x1FF` | ±16384 |
| GM6020 电压 | 1~4 | `0x204 + ID` | `0x1FF` | ±25000 |
| GM6020 电压 | 5~7 | `0x204 + ID` | `0x2FF` | ±25000 |
| GM6020 电流 | 1~4 | `0x204 + ID` | `0x1FE` | ±16384 |
| GM6020 电流 | 5~7 | `0x204 + ID` | `0x2FE` | ±16384 |

GM6020 电流模式要求固件版本不低于 1.0.11.2，并通过 RoboMaster Assistant
2.7 或更高版本开启电流环。

## 初始化

```cpp
Class_DJIMotor motor;

Struct_DJIMotor_Init_Config config{
    .hfdcan = &hfdcan1,
    .can_id = 1,
    .motor_type = Enum_DJIMotor_Type::GM6020,
    .close_loop = DJI_MOTOR_CURRENT_LOOP | DJI_MOTOR_SPEED_LOOP,
    .outer_loop = DJI_MOTOR_SPEED_LOOP,
    .current_pid = {
        .K_P = 0.5f, .Out_Max = 16000.0f, .D_T = 0.001f,
    },
    .speed_pid = {
        .K_P = 10.0f, .Out_Max = 16000.0f, .D_T = 0.001f,
    },
    .control_mode = Enum_DJIMotor_Control_Mode::CURRENT,
    .feedback_timeout_ms = 20,
};

if (motor.Init(config)) {
    motor.SetRef(90.0f);
}
```

非 GM6020 电机只接受 `CURRENT`。`gear_ratio <= 0` 时使用型号默认值：M2006 为
36、M3508 为官方标称约 19、GM6020 为 1。若实际机构或精度要求不同，应显式填写
实测或设计减速比。

## 控制周期

所有电机完成计算后只调用一次统一发送函数：

```cpp
motor_a.Control();
motor_b.Control();
motor_c.Control();
DJIMotor_SendAll();
```

`Control()` 只计算 PID 并更新对应共享帧槽，不触发 CAN 发布；`DJIMotor_SendAll()`
将每个已注册的控制组发布一次。项目的 `Can_Tx_Task` 随后通过 `BSP_CAN_SendPer()`
完成硬件发送。

`Disable()` 会立即清除对象所占共享槽。对象首次收到合法反馈前，以及超过
`feedback_timeout_ms` 没有反馈后，`Control()` 都会保持该槽为零并清除 PID 积分。

## 反馈量和单位

- `encoder`：协议原始 13 位转子编码器值，范围 0~8191。
- `rotor_angle`、`rotor_total_angle`：转子侧角度，单位 °。
- `rotor_speed`：转子侧滤波速度，单位 °/s。
- `output_angle`、`output_total_angle`、`output_speed`：上述转子量除以减速比。
- `current_raw`：协议返回的原始实际转矩电流值，不声明为安培。
- `temperature`：M3508 和 GM6020 的电机温度；C610 对应字节为空，因此保持 0。
- `online`、`last_feedback_tick`：watchdog 状态和最近反馈的 HAL 毫秒 tick。

反向配置作用于角度、速度以及控制输出的逻辑方向；`encoder` 和 `current_raw` 始终保留
协议原始值。内部电流环会根据反向配置转换 `current_raw` 的符号。

速度低通采用 `ROTOR_SPEED_LPF_ALPHA * old + (1-alpha) * measured`，当前 alpha
为 0.85，明确表示保留 85% 旧值。

## 冲突规则

`Init()` 同时检查同一 FDCAN 总线上的反馈 ID 和控制帧 slot。比如 M2006/M3508 ID 5
和 GM6020 ID 1 的反馈 ID 都是 `0x205`，在同一总线上注册时后者会返回 `false`。

## 多电机 Group

`Class_DJIMotor_Group` 只保存 1~4 个已经初始化的电机指针，不复制对象、不分配动态
内存，也不参与 PID、CAN 分组或发送。传入空洞、重复指针或尚未初始化的电机时，
`Init()` 返回 `false`。

### 四个 M3508 底盘

```cpp
Class_DJIMotor motor1;
Class_DJIMotor motor2;
Class_DJIMotor motor3;
Class_DJIMotor motor4;
Class_DJIMotor_Group chassis;

Struct_DJIMotor_Init_Config config{
    .hfdcan = &hfdcan1,
    .can_id = 1,
    .motor_type = Enum_DJIMotor_Type::M3508,
    .close_loop = DJI_MOTOR_CURRENT_LOOP | DJI_MOTOR_SPEED_LOOP,
    .outer_loop = DJI_MOTOR_SPEED_LOOP,
    .current_pid = {
        .K_P = 0.5f, .Out_Max = 16384.0f, .D_T = 0.001f,
    },
    .speed_pid = {
        .K_P = 10.0f, .Out_Max = 16000.0f, .D_T = 0.001f,
    },
    .control_mode = Enum_DJIMotor_Control_Mode::CURRENT,
};

bool ok = motor1.Init(config);
config.can_id = 2;
ok = motor2.Init(config) && ok;
config.can_id = 3;
ok = motor3.Init(config) && ok;
config.can_id = 4;
ok = motor4.Init(config) && ok;
ok = ok && chassis.Init(&motor1, &motor2, &motor3, &motor4);
```

控制周期：

```cpp
chassis.Update(v1, v2, v3, v4);
DJIMotor_SendAll();
```

`Update()` 等价于依次调用 `SetRef()` 和 `Control()`，内部不会调用
`DJIMotor_SendAll()`。需要分开设置目标与执行计算时，原有两个接口仍可使用。

### 底盘和云台同时存在

```cpp
chassis.Update(v1, v2, v3, v4);
gimbal.Update(yaw_ref, pitch_ref);

// 整个控制周期只调用一次，发送所有被更新的 DJI CAN 分组。
DJIMotor_SendAll();
```

Group 的 `Enable()` 和 `Disable()` 依次操作所有成员。Group 不拥有电机，因此成员电机
对象的生命周期必须长于 Group；推荐都使用静态或全局对象。

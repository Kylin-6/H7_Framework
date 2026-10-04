/**
 * @file Chassis.cpp
 * @brief 基于四个舵轮模块的 AGV 底盘应用，参考 Meta-Embedded-NG 移植。
 *
 * 运动学与舵向最短路径规则来自 MIT 许可证下的 Meta-Embedded-NG
 * application/chassis，实现已适配本工程 Class_DJIMotor 接口。机械参数仍是
 * 待实车标定值；双板构建将此模块放在底盘板。
 * @todo vx/vy/wz 的实车物理正方向须按电机安装和坐标系标定，不能仅由数组顺序推断。
 * @todo 舵向依赖 output_total_angle；增量编码器上电不提供绝对输出轴零位，
 *       需绝对编码器、寻零流程或已知上电姿态。
 */

#include "Chassis.h"
#include "Chassis_Config.h"
#include "board_config.h"
#include "message_center.h"

static constexpr uint64_t CHASSIS_COMMAND_MAX_AGE_US = 100000U;

#include "dji_motor.h"
#include "fdcan.h"

#include <cmath>

namespace
{
struct ChassisContext
{
    Publisher<ChassisFeedback> feedback_publisher{MessageCenter::Chassis_Feedback_Topic};
    ChassisCmd command{};
    ChassisFeedback feedback{};
    uint8_t feedback_divider = 0U;
    Class_DJIMotor wheel_motor[4];
    Class_DJIMotor steer_motor[4];
    Struct_DJIMotor_Motion_Snapshot wheel_snapshot[4];
    Struct_DJIMotor_Motion_Snapshot steer_snapshot[4];
    Class_DJIMotor_Group wheel_group;
    Class_DJIMotor_Group steer_group;
    bool initialized = false;
    int8_t wheel_direction[4] = {1, 1, 1, 1};
};

ChassisContext ctx;
} // namespace

/**
 * @brief 将机构配置转换为 DJI 驱动 PID 参数，计算周期固定为 1 ms。
 * @note 轮速/舵速环输出协议指令，舵角环输出 rad/s，不能混用增益和限幅单位。
 */
static PID_InitTypeDef Chassis_MakePID(const ChassisPidConfig& config)
{
    PID_InitTypeDef pid{};
    pid.K_P = config.kp;
    pid.K_I = config.ki;
    pid.K_D = config.kd;
    pid.I_Out_Max = config.integral_limit;
    pid.Out_Max = config.output_limit;
    pid.D_T = 0.001f;
    return pid;
}

/**
 * @brief 将底盘速度分解为四轮目标，并按最短舵向选择轮速正反方向。
 * @param wheel_target_rad_s 输出四个行走轮的输出轴角速度，单位 rad/s。
 * @param steer_target_rad 输出四个舵向电机的累计角度目标，单位 rad。
 * @note 使用本周期命令和舵向快照；轮索引、方向及机械零位须与配置一致。
 */
static void Chassis_CalculateTargets(float wheel_target_rad_s[4],
                                     float steer_target_rad[4])
{
    /* 四轮位置的旋转项为 ±wz·半宽/半长；符号按下方轮索引数组固定。
       物理前/左和正转方向必须由实车接线及坐标标定确认。 */
    const float vx_m_s = ctx.command.velocity_x_m_s;
    const float vy_m_s = ctx.command.velocity_y_m_s;
    const float wz_rad_s = ctx.command.angular_velocity_rad_s;
    const float wheel_vx[4] = {
        vx_m_s + wz_rad_s * kChassisConfig.half_width_m,
        vx_m_s + wz_rad_s * kChassisConfig.half_width_m,
        vx_m_s - wz_rad_s * kChassisConfig.half_width_m,
        vx_m_s - wz_rad_s * kChassisConfig.half_width_m,
    };
    const float wheel_vy[4] = {
        vy_m_s + wz_rad_s * kChassisConfig.half_length_m,
        vy_m_s - wz_rad_s * kChassisConfig.half_length_m,
        vy_m_s - wz_rad_s * kChassisConfig.half_length_m,
        vy_m_s + wz_rad_s * kChassisConfig.half_length_m,
    };

    for (uint8_t index = 0; index < 4; ++index)
    {
        const float velocity_m_s = std::sqrt(wheel_vx[index] * wheel_vx[index] +
                                             wheel_vy[index] * wheel_vy[index]);
        const float current_angle_rad =
            ctx.steer_snapshot[index].output_total_angle;
        if (velocity_m_s < kChassisConfig.stop_speed_m_s) // 近零轮速不确定方向，保持舵角，避免微小速度输入造成转向跳变。
        {
            // 近零轮速时保持当前舵角，避免 atan2 的方向随微小输入跳变。
            wheel_target_rad_s[index] = 0.0f;
            steer_target_rad[index] = current_angle_rad;
            continue;
        }

        const float target_angle_rad = std::atan2(wheel_vy[index], wheel_vx[index]) +
                                       kChassisConfig.steer_offset_rad[index];
        float difference_rad = std::remainder(target_angle_rad - current_angle_rad,
                                              2.0f * kPiRad);
        /* remainder 将误差压到 [-π, π]；超过 ±π/2 时舵角少转 π、轮速取反。 */
        if (difference_rad > kPiRad / 2.0f) // 正向舵角差超过 90°，少转 180°并反转行走轮可保持同一速度向量。
        {
            difference_rad -= kPiRad;
            ctx.wheel_direction[index] = -1;
        }
        else if (difference_rad < -kPiRad / 2.0f) // 负向舵角差超过 90°，同样通过转向补偿与轮速反转缩短转向路径。
        {
            difference_rad += kPiRad;
            ctx.wheel_direction[index] = -1;
        }
        else
        {
            ctx.wheel_direction[index] = 1;
        }

        steer_target_rad[index] = current_angle_rad + difference_rad;
        // 线速度除以轮半径得到输出轴 rad/s；后续闭环由 DJI 驱动的现有 PID 执行。
        wheel_target_rad_s[index] = (velocity_m_s / kChassisConfig.wheel_radius_m) *
                                    ctx.wheel_direction[index];
    }
}

/**
 * @brief 用轮速与舵角估算底盘速度，并聚合八台电机 online/ready。
 * @note 结果是运动学估计，不是融合里程计；滑移和零位偏差会影响估计。
 *       使用更新入口读取的快照，反馈平滑在 1 kHz 执行，消息以 100 Hz 发布。
 */
static void Chassis_UpdateFeedback(void)
{
    float wheel_vx[4];
    float wheel_vy[4];
    bool online = true;
    bool ready = true;
    for (uint8_t index = 0; index < 4; ++index)
    {
        const float heading_rad =
            ctx.steer_snapshot[index].output_total_angle -
            kChassisConfig.steer_offset_rad[index];
        const float linear_speed_m_s =
            ctx.wheel_snapshot[index].output_speed *
            kChassisConfig.wheel_radius_m;
        wheel_vx[index] = linear_speed_m_s * std::cos(heading_rad);
        wheel_vy[index] = linear_speed_m_s * std::sin(heading_rad);
        online = online && ctx.wheel_snapshot[index].online &&
                 ctx.steer_snapshot[index].online;
        ready = ready && ctx.wheel_snapshot[index].ready &&
                ctx.steer_snapshot[index].ready;
    }

    const float vx = (wheel_vx[0] + wheel_vx[1] + wheel_vx[2] + wheel_vx[3]) * 0.25f;
    const float vy = (wheel_vy[0] + wheel_vy[1] + wheel_vy[2] + wheel_vy[3]) * 0.25f;
    const float wz_x = ((wheel_vx[0] - wheel_vx[2]) +
                        (wheel_vx[1] - wheel_vx[3])) /
                       (4.0f * kChassisConfig.half_width_m);
    const float wz_y = ((wheel_vy[0] - wheel_vy[1]) +
                        (wheel_vy[3] - wheel_vy[2])) /
                       (4.0f * kChassisConfig.half_length_m);

    // 一阶平滑 y += alpha * (x - y)，在 1 kHz 控制周期更新；100 Hz 仅是发布频率。
    // 初始输出沿用初始化时的零值，feedback_alpha 是每个控制周期的权重。
    ctx.feedback.velocity_x_m_s += kChassisConfig.feedback_alpha *
                                   (vx - ctx.feedback.velocity_x_m_s);
    ctx.feedback.velocity_y_m_s += kChassisConfig.feedback_alpha *
                                   (vy - ctx.feedback.velocity_y_m_s);
    ctx.feedback.angular_velocity_rad_s += kChassisConfig.feedback_alpha *
                                           (0.5f * (wz_x + wz_y) - ctx.feedback.angular_velocity_rad_s);
    // 快照在本周期使能请求之前读取，enabled 可能滞后一周期；不代表 CAN 已发送。
    ctx.feedback.enabled = ctx.command.mode != ChassisMode::ZERO_FORCE && ready;
    ctx.feedback.online = online;
}

/**
 * @brief 启动阶段注册八台电机，再绑定两个四电机发送组。
 * @return 全部电机注册与发送组绑定成功时为 true。
 * @note 仅调用一次；成功后请求零输出，不执行机械寻零，不等待电机反馈。
 */
bool Chassis_Init(void)
{
    ctx.command = {};
    ctx.feedback = {};
    ctx.feedback_divider = 0U;

    Struct_DJIMotor_Init_Config wheel_config{};
    wheel_config.hfdcan = BoardConfig_Get().chassis_wheel_bus;
    wheel_config.motor_type = Enum_DJIMotor_Type::M3508;
    wheel_config.close_loop = DJI_MOTOR_SPEED_LOOP;
    wheel_config.outer_loop = DJI_MOTOR_SPEED_LOOP;
    // 速度环输入现为 rad/s；以下增益来源未标定，需实车重新整定。
    wheel_config.speed_pid = Chassis_MakePID(kChassisConfig.wheel_speed_pid);

    Struct_DJIMotor_Init_Config steer_config{};
    steer_config.hfdcan = BoardConfig_Get().chassis_steer_bus;
    steer_config.motor_type = Enum_DJIMotor_Type::M3508;
    steer_config.close_loop = DJI_MOTOR_ANGLE_LOOP | DJI_MOTOR_SPEED_LOOP;
    steer_config.outer_loop = DJI_MOTOR_ANGLE_LOOP;
    // 角度环输出为 rad/s：原 200/1000 deg/s 限幅作物理等效转换。
    // Kp/Ki 和舵轮速度环增益没有可信实车来源，启用前均需重新整定。
    steer_config.angle_pid = Chassis_MakePID(kChassisConfig.steer_angle_pid);
    steer_config.speed_pid = Chassis_MakePID(kChassisConfig.steer_speed_pid);

    bool initialized = true;
    for (uint8_t index = 0; index < 4; ++index)
    {
        wheel_config.can_id = kChassisConfig.motor_id[index];
        steer_config.can_id = kChassisConfig.motor_id[index];
        initialized = ctx.wheel_motor[index].Init(wheel_config) && initialized;
        initialized = ctx.steer_motor[index].Init(steer_config) && initialized;
    }
    initialized = initialized && ctx.wheel_group.Init(
                                     &ctx.wheel_motor[0], &ctx.wheel_motor[1],
                                     &ctx.wheel_motor[2], &ctx.wheel_motor[3]);
    initialized = initialized && ctx.steer_group.Init(
                                     &ctx.steer_motor[0], &ctx.steer_motor[1],
                                     &ctx.steer_motor[2], &ctx.steer_motor[3]);
    ctx.initialized = initialized;
    if (initialized) // 仅在全部电机与发送组初始化成功后提交零输出；不代表设备已经在线。
    {
        (void) ctx.wheel_group.RequestEnabled(false);
        (void) ctx.steer_group.RequestEnabled(false);
    }
    return initialized;
}

/**
 * @brief 1 kHz 读取新鲜命令、计算舵轮目标、提交组输出并更新反馈。
 * @note 命令超过 100 ms 时默认 ZERO_FORCE；设备反馈超时由 DJI 驱动逐电机处理。
 *       此处不仲裁输入、不解析 CAN、不新增模式控制器；反馈按十个周期分频发布。
 */
void Chassis_Update(void)
{
    /* 仅在命令 Topic 仍新鲜时沿用目标；过期后使用默认 ZERO_FORCE 关闭输出。 */
    ctx.command = {};
    // 命令需存在且年龄不超过 100 ms；双板由 Transport 发布，本地由 RobotCmd 发布。
    (void) MessageCenter::Chassis_Command_Topic.ReadFresh(
        ctx.command, CHASSIS_COMMAND_MAX_AGE_US);

    if (ctx.initialized) // 初始化失败时不访问正常组控制流程，反馈仍按周期发布。
    {
        for (uint8_t index = 0U; index < 4U; ++index)
        {
            ctx.wheel_snapshot[index] = ctx.wheel_motor[index].GetMotionSnapshot();
            ctx.steer_snapshot[index] = ctx.steer_motor[index].GetMotionSnapshot();
        }
        const bool enabled = ctx.command.mode != ChassisMode::ZERO_FORCE;
        (void) ctx.wheel_group.RequestEnabled(enabled);
        (void) ctx.steer_group.RequestEnabled(enabled);
        if (enabled) // 非 ZERO_FORCE 才计算运动目标；实际反馈超时保护由驱动逐电机执行。
        {
            float wheel_target_rad_s[4];
            float steer_target_rad[4];
            Chassis_CalculateTargets(wheel_target_rad_s, steer_target_rad);
            ctx.wheel_group.Control(wheel_target_rad_s[0], wheel_target_rad_s[1],
                                    wheel_target_rad_s[2], wheel_target_rad_s[3]);
            ctx.steer_group.Control(steer_target_rad[0], steer_target_rad[1],
                                    steer_target_rad[2], steer_target_rad[3]);
        }
        Chassis_UpdateFeedback();
    }

    /* 电机控制按 1 kHz 执行，反馈消息按 100 Hz 发布。 */
    ctx.feedback_divider++;
    if (ctx.feedback_divider >= 10U) // 1 kHz 下每十周期发布一次，反馈采样和平滑仍是 1 kHz。
    {
        ctx.feedback_divider = 0U;
        ctx.feedback_publisher.Publish(ctx.feedback);
    }
}

/**
 * @file dmmotor.h
 * @brief 达妙电机配置、控制接口与反馈数据。
 * @author Kylin-6
 * @note 原始驱动由 Kylin-6 在 PR #4 贡献，后续适配由 zzm 维护。
 */

#ifndef DMMOTOR_H
#define DMMOTOR_H

#include "bsp_can.h"
#include "daemon.h"

#include <stdint.h>

enum class Enum_DMMotor_Mode : uint8_t
{
    MIT = 1U,            ///< MIT 五参数控制
    POSITION_SPEED = 2U, ///< 位置-速度控制
    SPEED = 3U,          ///< 速度控制
    FORCE_POSITION = 4U, ///< 力位混合控制
};

struct Struct_DMMotor_Feedback
{
    uint8_t state = 0;              ///< 协议状态码，高四位解码结果。
    float position = 0.0f;          ///< 单圈位置，rad，已应用 reverse。
    float total_position = 0.0f;    ///< 按协议位置量程展开的累计位置，rad。
    float velocity = 0.0f;          ///< 速度，rad/s，已应用 reverse。
    float torque = 0.0f;            ///< 转矩，N*m，已应用 reverse。
    float mos_temperature = 0.0f;   ///< MOS 温度，摄氏度。
    float rotor_temperature = 0.0f; ///< 转子温度，摄氏度。
};

/** 同一时刻取得的运动反馈与状态；online 按最近反馈时间判定，不等待 StatusTask。 */
struct Struct_DMMotor_Snapshot
{
    Struct_DMMotor_Feedback feedback{};
    bool online = false;
    bool requested_enabled = false;
    bool actual_enabled = false;
    bool fault = false; ///< 新鲜反馈报告非失能、非使能的协议故障状态。
    bool ready = false;
};

class Class_DMMotor
{
public:
    /** 初始化 CAN 参数、反馈回调，并把在线守护器注册到 DaemonManager。 */
    bool Init(FDCAN_HandleTypeDef *hfdcan,
              uint8_t can_id,
              uint16_t master_id,
              Enum_DMMotor_Mode mode,
              bool reverse = false,
              float position_max = 12.5f,
              float velocity_max = 30.0f,
              float torque_max = 10.0f);
    /** 相同请求无动作并返回 true；使能边沿提交 Enable，失能边沿发布安全目标并提交 Disable。
     *  返回本次边沿所需提交的结果，100 Hz ServiceAll 依据反馈纠正，不代表电机执行。
     */
    bool RequestEnabled(bool enabled);
    /** @name 离散命令
     *  @brief true 仅表示命令已进入软件 FIFO，不代表电机执行或确认；false 时由上层决定重试。
     */
    ///@{
    bool ClearError();
    bool SetZeroPosition();
    /** 参数非法、其他模式待应答或入队失败均返回 false。 */
    bool SetMode(Enum_DMMotor_Mode mode);
    ///@}

    /** @name 连续控制目标
     *  @brief 更新对应 CAN 周期槽；同一总线和 ID 的旧目标会被最新值覆盖。
     *         返回值只表示软件槽是否接受目标，不代表硬件执行。
     */
    ///@{
    bool SetMIT(float position_rad,
                float velocity_rad_s,
                float kp,
                float kd,
                float torque_nm);
    bool SetPositionSpeed(float position_rad, float velocity_rad_s);
    bool SetSpeed(float speed_rad_s);
    bool SetForcePosition(float position_rad,
                          float velocity_limit_rad_s,
                          float current_limit_ratio);
    bool SetTorque(float torque_nm);

    ///@}

    /** 获取一致快照；actual_enabled 仅表示最近反馈的协议状态。 */
    Struct_DMMotor_Snapshot GetFeedbackSnapshot() const;
    /** 由现有 100 Hz StatusTask 调用，维护期望状态的协议命令。 */
    static void ServiceAll();

    /** 最近 100 ms 内收到过合法运动反馈时返回 true。 */
    bool IsOnline() const;
    /** 最近一帧合法反馈中的协议状态为“已使能”时返回 true。 */
    bool IsEnabled() const;
    /** 当前数据是否可用于控制；达妙驱动中等价于 IsOnline()。 */
    bool IsDataValid() const;
    /** 请求使能、在线且协议已使能时返回 true。 */
    bool IsHealthy() const;
    /** 提供只读守护器状态，供诊断层读取离线时间和状态跃迁。 */
    const Daemon &GetDaemon() const;

    Struct_DMMotor_Feedback feedback;

private:
    static void FeedbackCallback(FDCAN_HandleTypeDef *hfdcan,
                                 uint32_t id,
                                 uint8_t *data,
                                 uint32_t len,
                                 void *context);
    bool SendModeCommand(uint8_t command);
    bool Publish(const Struct_CAN_Tx_Msg &message);
    bool PublishSafeOutput();
    uint32_t ControlId() const;

    FDCAN_HandleTypeDef *hfdcan = nullptr;
    uint8_t can_id = 0U;
    uint16_t master_id = 0U;
    volatile Enum_DMMotor_Mode mode = Enum_DMMotor_Mode::MIT;
    volatile uint32_t requested_mode = 0;
    volatile bool mode_pending = false;
    volatile uint64_t mode_request_timestamp_us = 0;
    bool reverse = false;
    float position_max = 12.5f;
    float velocity_max = 30.0f;
    float torque_max = 10.0f;
    volatile bool requested_enabled = false;
    volatile bool lifecycle_requested = false;
    bool service_registered = false;
    Class_DMMotor *service_next = nullptr;
    static Class_DMMotor *service_head;
    uint64_t last_feedback_us = 0;
    bool feedback_initialized = false;
    float last_position = 0.0f;
    int32_t total_round = 0;
    Daemon feedback_daemon{100U}; ///< 只跟踪合法运动反馈的在线状态。
};

#endif

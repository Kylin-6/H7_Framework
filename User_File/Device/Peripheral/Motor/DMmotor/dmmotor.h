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
    /** @name 离散命令
     *  @brief true 仅表示命令已进入软件 FIFO，不代表电机执行或确认；false 时由上层决定重试。
     */
    ///@{
    bool Enable();
    bool Disable();
    bool ClearError();
    bool SetZeroPosition();
    /** 参数非法、其他模式待应答或入队失败均返回 false。 */
    bool SetMode(Enum_DMMotor_Mode mode);
    ///@}

    /** @name 连续控制目标
     *  @brief 更新对应 CAN 周期槽；同一总线和 ID 的旧目标会被最新值覆盖。
     */
    ///@{
    void SetMIT(float position_rad,
                float velocity_rad_s,
                float kp,
                float kd,
                float torque_nm);
    void SetPositionSpeed(float position_rad, float velocity_rad_s);
    void SetSpeed(float speed_rad_s);
    void SetForcePosition(float position_rad,
                          float velocity_limit_rad_s,
                          float current_limit_ratio);
    void SetTorque(float torque_nm);
    ///@}

    /** 最近 100 ms 内收到过合法运动反馈时返回 true。 */
    bool IsOnline() const;
    /** 最近一帧合法反馈中的协议状态为“已使能”时返回 true。 */
    bool IsEnabled() const;
    /** 当前数据是否可用于控制；达妙驱动中等价于 IsOnline()。 */
    bool IsDataValid() const;
    /** 同时在线且协议已使能时返回 true。 */
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
    /** Daemon 的 Online -> Offline 跃迁回调；每次跃迁最多提交一次使能帧。 */
    static void OfflineCallback(void *owner);
    bool SendModeCommand(uint8_t command);
    void Publish(const Struct_CAN_Tx_Msg &message);
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
    bool feedback_initialized = false;
    float last_position = 0.0f;
    int32_t total_round = 0;
    Daemon feedback_daemon{100U, OfflineCallback, this}; ///< 仅合法运动反馈喂狗；掉线跃迁时尝试一次使能。
};

#endif

/**
 * @file daemon.h
 * @brief 设备在线状态守护器与静态守护器管理器。
 * @details
 * Daemon 只负责记录合法通信的最后时间，并判断设备是否超时离线；不负责
 * 电机停机、机器人安全策略、日志、消息路由或设备重启。
 */

#ifndef SYSTEM_DAEMON_H
#define SYSTEM_DAEMON_H

#include <cstdint>

/** @brief 一次 Check() 所观察到的在线状态跃迁。 */
enum class DaemonTransition : uint8_t
{
    None = 0U,        ///< 状态没有变化。
    OnlineToOffline, ///< 设备本周期首次被判定为离线。
    OfflineToOnline, ///< 设备离线后首次收到合法反馈。
};

/**
 * @brief 单个设备的非阻塞在线状态监视器。
 * @note 新建对象在收到第一帧合法数据前保持 Offline。
 * @note Feed() 可以在 ISR 或任务中调用；Check() 由低频 StatusTask 调用。
 */
class Daemon
{
public:
    /** @param timeout_ms 设备从最后一次合法反馈开始允许的最大静默时间。 */
    explicit Daemon(uint32_t timeout_ms);

    /** @brief 收到一帧确认有效的数据后喂狗，并立即把设备标记为在线。 */
    void Feed();

    /**
     * @brief 根据当前时间检查一次超时和状态变化。
     * @return 本次检查产生的跃迁；稳定状态返回 None。
     */
    DaemonTransition Check();

    /** @brief 获取当前在线状态。 */
    bool IsOnline() const;

    /** @brief 获取最近一次有效 Feed() 的毫秒时间戳。 */
    uint32_t LastFeedMs() const;

    /** @brief 获取本次离线持续时间；在线时返回 0。 */
    uint32_t OfflineDurationMs() const;

    /** @brief 获取最近一次 Check() 保存的状态跃迁。 */
    DaemonTransition LastTransition() const;

private:
    const uint32_t timeout_ms_; ///< 当前设备独立的超时门限。
    uint32_t last_feed_ms_ = 0U; ///< 最近一次有效通信时间。
    uint32_t offline_since_ms_ = 0U; ///< 最近一次离线跃迁发生时间。
    bool online_ = false; ///< 当前在线状态。
    bool online_transition_pending_ = false; ///< 等待 Check() 报告上线跃迁。
    DaemonTransition last_transition_ = DaemonTransition::None; ///< 最近检查结果。
};

/**
 * @brief 固定容量的 Daemon 注册表，由 StatusTask 统一遍历。
 * @note 设备只在初始化阶段注册；不使用动态内存，也不提供运行期注销。
 */
class DaemonManager
{
public:
    /** @brief 注册一个静态生命周期的 Daemon；重复注册同一对象仍返回成功。 */
    static bool Register(Daemon &daemon);

    /** @brief 顺序检查当前已经注册的全部 Daemon。 */
    static void CheckAll();

private:
    static constexpr uint8_t MAX_DAEMONS = 32U; ///< 当前框架允许的最大设备数。
    static Daemon *daemons_[MAX_DAEMONS]; ///< 只保存外部静态对象地址，不拥有对象。
    static uint8_t daemon_count_; ///< 已注册对象数量，只增不减。
};

#endif

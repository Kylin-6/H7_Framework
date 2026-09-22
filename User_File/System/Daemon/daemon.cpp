/**
 * @file daemon.cpp
 * @brief Daemon 在线检测与固定容量管理器实现。
 * @details
 * 固定注册表和“设备直接喂狗”思路参考 MIT 许可的 basic_framework 与
 * Meta-Embedded-NG；本实现改用绝对时间戳、静态内存和单次状态跃迁。
 */

#include "daemon.h"

#include "stm32h7xx.h"
#include "sys_timestamp.h"

/* 管理器只保存指针，Daemon 对象由具体 Device 静态持有。 */
Daemon *DaemonManager::daemons_[DaemonManager::MAX_DAEMONS] = {};
uint8_t DaemonManager::daemon_count_ = 0U;

namespace
{
/** @brief 复用工程统一微秒时间源，并截取为可自然回绕的 uint32_t 毫秒时间。 */
uint32_t DaemonNowMs()
{
    return static_cast<uint32_t>(SYS_Timestamp_Get_Microsecond() / 1000ULL);
}

/**
 * @brief 进入极短 PRIMASK 临界区。
 * @return 进入前的 PRIMASK，退出时必须原样恢复，支持嵌套调用场景。
 */
uint32_t DaemonEnterCritical()
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    __DMB();
    return primask;
}

/** @brief 恢复进入临界区前的中断状态。 */
void DaemonExitCritical(uint32_t primask)
{
    __DMB();
    __set_PRIMASK(primask);
}
}

Daemon::Daemon(uint32_t timeout_ms,
               OfflineCallback offline_callback,
               void *owner)
    : timeout_ms_(timeout_ms),
      offline_callback_(offline_callback),
      owner_(owner)
{
}

void Daemon::Feed()
{
    // 时间读取放在临界区外，临界区内只更新少量标量，缩短关中断时间。
    const uint32_t now_ms = DaemonNowMs();
    const uint32_t primask = DaemonEnterCritical();

    last_feed_ms_ = now_ms;
    if (!online_)
    {
        // Feed 立即恢复在线状态；跃迁留给下一次 Check() 统一报告一次。
        online_ = true;
        online_transition_pending_ = true;
        last_transition_ = DaemonTransition::OfflineToOnline;
    }

    DaemonExitCritical(primask);
}

DaemonTransition Daemon::Check()
{
    const uint32_t now_ms = DaemonNowMs();
    const uint32_t primask = DaemonEnterCritical();
    DaemonTransition transition = DaemonTransition::None;
    OfflineCallback callback = nullptr;
    void *owner = nullptr;

    // 无符号减法天然支持 uint32_t 毫秒计数器回绕。
    if (online_ && (now_ms - last_feed_ms_) >= timeout_ms_)
    {
        online_ = false;
        online_transition_pending_ = false;
        offline_since_ms_ = now_ms;
        transition = DaemonTransition::OnlineToOffline;
        callback = offline_callback_;
        owner = owner_;
    }
    else if (online_transition_pending_)
    {
        // 清除 pending，保证 OfflineToOnline 不会在每个检查周期重复出现。
        online_transition_pending_ = false;
        transition = DaemonTransition::OfflineToOnline;
    }

    last_transition_ = transition;
    DaemonExitCritical(primask);

    // 设备恢复策略在临界区外执行，避免关中断时调用 RTOS/CAN 接口。
    if (callback != nullptr)
    {
        callback(owner);
    }
    return transition;
}

bool Daemon::IsOnline() const
{
    const uint32_t primask = DaemonEnterCritical();
    const bool online = online_;
    DaemonExitCritical(primask);
    return online;
}

uint32_t Daemon::LastFeedMs() const
{
    const uint32_t primask = DaemonEnterCritical();
    const uint32_t last_feed_ms = last_feed_ms_;
    DaemonExitCritical(primask);
    return last_feed_ms;
}

uint32_t Daemon::OfflineDurationMs() const
{
    const uint32_t now_ms = DaemonNowMs();
    const uint32_t primask = DaemonEnterCritical();
    const bool online = online_;
    const uint32_t offline_since_ms = offline_since_ms_;
    DaemonExitCritical(primask);
    return online ? 0U : now_ms - offline_since_ms;
}

DaemonTransition Daemon::LastTransition() const
{
    const uint32_t primask = DaemonEnterCritical();
    const DaemonTransition transition = last_transition_;
    DaemonExitCritical(primask);
    return transition;
}

bool DaemonManager::Register(Daemon &daemon)
{
    const uint32_t primask = DaemonEnterCritical();

    for (uint8_t index = 0U; index < daemon_count_; ++index)
    {
        // 初始化代码重复调用时保持幂等，不在数组中插入第二个相同指针。
        if (daemons_[index] == &daemon)
        {
            DaemonExitCritical(primask);
            return true;
        }
    }

    if (daemon_count_ >= MAX_DAEMONS)
    {
        DaemonExitCritical(primask);
        return false;
    }

    daemons_[daemon_count_] = &daemon;
    daemon_count_++;
    DaemonExitCritical(primask);
    return true;
}

void DaemonManager::CheckAll()
{
    // 注册表运行期只追加且不删除；临界区只读取计数，实际 Check 在区外执行。
    const uint32_t primask = DaemonEnterCritical();
    const uint8_t daemon_count = daemon_count_;
    DaemonExitCritical(primask);

    for (uint8_t index = 0U; index < daemon_count; ++index)
    {
        daemons_[index]->Check();
    }
}

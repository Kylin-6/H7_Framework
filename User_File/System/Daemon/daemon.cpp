#include "daemon.h"

#include "stm32h7xx.h"
#include "sys_timestamp.h"

// The fixed registry and direct device-feed model are informed by the
// MIT-licensed basic_framework / Meta-Embedded-NG daemon modules. This version
// uses no heap, absolute timestamps, and edge-triggered state transitions.

Daemon *DaemonManager::daemons_[DaemonManager::MAX_DAEMONS] = {};
uint8_t DaemonManager::daemon_count_ = 0U;

namespace
{
uint32_t DaemonNowMs()
{
    return static_cast<uint32_t>(SYS_Timestamp_Get_Microsecond() / 1000ULL);
}

uint32_t DaemonEnterCritical()
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    __DMB();
    return primask;
}

void DaemonExitCritical(uint32_t primask)
{
    __DMB();
    __set_PRIMASK(primask);
}
}

Daemon::Daemon(uint32_t timeout_ms)
    : timeout_ms_(timeout_ms)
{
}

void Daemon::Feed()
{
    const uint32_t now_ms = DaemonNowMs();
    const uint32_t primask = DaemonEnterCritical();

    last_feed_ms_ = now_ms;
    if (!online_)
    {
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

    if (online_ && (now_ms - last_feed_ms_) >= timeout_ms_)
    {
        online_ = false;
        online_transition_pending_ = false;
        offline_since_ms_ = now_ms;
        transition = DaemonTransition::OnlineToOffline;
    }
    else if (online_transition_pending_)
    {
        online_transition_pending_ = false;
        transition = DaemonTransition::OfflineToOnline;
    }

    last_transition_ = transition;
    DaemonExitCritical(primask);
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
    const uint32_t primask = DaemonEnterCritical();
    const uint8_t daemon_count = daemon_count_;
    DaemonExitCritical(primask);

    for (uint8_t index = 0U; index < daemon_count; ++index)
    {
        daemons_[index]->Check();
    }
}

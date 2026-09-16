#ifndef SYSTEM_DAEMON_H
#define SYSTEM_DAEMON_H

#include <cstdint>

enum class DaemonTransition : uint8_t
{
    None = 0U,
    OnlineToOffline,
    OfflineToOnline,
};

/**
 * @brief Non-blocking latest-feed monitor for one device.
 *
 * A newly constructed daemon is offline until the first valid Feed().
 * Feed() may be called from an ISR or task; Check() is intended for the
 * low-frequency system status path.
 */
class Daemon
{
public:
    explicit Daemon(uint32_t timeout_ms);

    void Feed();
    DaemonTransition Check();

    bool IsOnline() const;
    uint32_t LastFeedMs() const;
    uint32_t OfflineDurationMs() const;
    DaemonTransition LastTransition() const;

private:
    const uint32_t timeout_ms_;
    uint32_t last_feed_ms_ = 0U;
    uint32_t offline_since_ms_ = 0U;
    bool online_ = false;
    bool online_transition_pending_ = false;
    DaemonTransition last_transition_ = DaemonTransition::None;
};

/**
 * @brief Fixed-capacity registry checked by the existing periodic task.
 * @note Register devices during their initialization. There is no dynamic
 *       allocation and registered entries are never removed.
 */
class DaemonManager
{
public:
    static bool Register(Daemon &daemon);
    static void CheckAll();

private:
    static constexpr uint8_t MAX_DAEMONS = 32U;
    static Daemon *daemons_[MAX_DAEMONS];
    static uint8_t daemon_count_;
};

#endif

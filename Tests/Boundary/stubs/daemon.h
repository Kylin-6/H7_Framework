#pragma once

#include <stdint.h>

enum class DaemonTransition : uint8_t
{
    None = 0U,
    OnlineToOffline,
    OfflineToOnline,
};

class Daemon
{
public:
    using OfflineCallback = void (*)(void *owner);
    explicit Daemon(uint32_t, OfflineCallback = nullptr, void * = nullptr) {}
    void Feed() { online_ = true; }
    DaemonTransition Check() { return DaemonTransition::None; }
    bool IsOnline() const { return online_; }
    uint32_t LastFeedMs() const { return 0U; }
    uint32_t OfflineDurationMs() const { return 0U; }
    DaemonTransition LastTransition() const { return DaemonTransition::None; }

private:
    bool online_ = false;
};

class DaemonManager
{
public:
    static bool Register(Daemon &) { return true; }
    static void CheckAll() {}
};

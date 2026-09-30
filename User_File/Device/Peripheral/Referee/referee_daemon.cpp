#include "referee_daemon.h"
#include "daemon.h"

namespace
{
Daemon referee_daemon{REFEREE_OFFLINE_TIMEOUT_MS};
Daemon vtm_daemon{VTM_OFFLINE_TIMEOUT_MS};
}

extern "C" bool RefereeDaemonRegister(void)
{
    return DaemonManager::Register(referee_daemon);
}
extern "C" void RefereeDaemonFeed(void) { referee_daemon.Feed(); }
extern "C" bool RefereeDaemonIsOnline(void) { return referee_daemon.IsOnline(); }

extern "C" bool VTMDaemonRegister(void)
{
    return DaemonManager::Register(vtm_daemon);
}
extern "C" void VTMDaemonFeed(void) { vtm_daemon.Feed(); }
extern "C" bool VTMDaemonIsOnline(void) { return vtm_daemon.IsOnline(); }

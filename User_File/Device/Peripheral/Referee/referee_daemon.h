#ifndef REFEREE_DAEMON_H
#define REFEREE_DAEMON_H

#include <stdbool.h>

#define REFEREE_OFFLINE_TIMEOUT_MS 500U
/* 暂定值，需根据实际图传帧周期上车验证。 */
#define VTM_OFFLINE_TIMEOUT_MS 300U

#ifdef __cplusplus
extern "C" {
#endif

/* C 解析器到静态 C++ Daemon 的薄接口，不承担协议或业务策略。 */
bool RefereeDaemonRegister(void);
void RefereeDaemonFeed(void);
bool RefereeDaemonIsOnline(void);
bool VTMDaemonRegister(void);
void VTMDaemonFeed(void);
bool VTMDaemonIsOnline(void);

#ifdef __cplusplus
}
#endif
#endif

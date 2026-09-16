#ifndef APPLICATION_H
#define APPLICATION_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 在调度器启动前一次性注册全部应用层消息端点。
 * @return 全部注册成功返回 true；任一模块失败返回 false。
 * @note 调用方必须检查返回值，避免控制链因注册失败而静默失效。
 */
bool Application_RegisterTopics(void);

#ifdef __cplusplus
}
#endif

#endif

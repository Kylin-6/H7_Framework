#ifndef APPLICATION_H
#define APPLICATION_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Register all application message endpoints before the scheduler starts. */
bool Application_RegisterTopics(void);

#ifdef __cplusplus
}
#endif

#endif

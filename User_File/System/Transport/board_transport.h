#ifndef BOARD_TRANSPORT_H
#define BOARD_TRANSPORT_H

#include "message_types.h"

/** Implemented only by the selected board's fixed transport binding. */
void BoardTransport_Init(void);
void BoardTransport_Poll(void);
void BoardTransport_SendChassis(const ChassisCmd &command);

#endif

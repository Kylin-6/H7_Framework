#ifndef MESSAGE_CENTER_H
#define MESSAGE_CENTER_H

#include "event_queue.h"
#include "message_types.h"
#include "topic.h"

namespace MessageCenter
{
extern Topic<INS_State> INS_State_Topic;
extern Topic<GimbalCmd> Gimbal_Command_Topic;
extern Topic<ChassisCmd> Chassis_Command_Topic;
extern Topic<ShootCmd> Shoot_Command_Topic;
extern Topic<GimbalFeedback> Gimbal_Feedback_Topic;
extern Topic<ChassisFeedback> Chassis_Feedback_Topic;
extern Topic<ShootFeedback> Shoot_Feedback_Topic;
extern EventQueue<ShootEvent, 8U> Shoot_Event_Queue;
}

#endif

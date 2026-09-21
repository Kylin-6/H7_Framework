#include "message_center.h"

namespace MessageCenter
{
Topic<INS_State> INS_State_Topic;
Topic<GimbalCmd> Gimbal_Command_Topic;
Topic<ChassisCmd> Chassis_Command_Topic;
Topic<ShootCmd> Shoot_Command_Topic;
Topic<GimbalFeedback> Gimbal_Feedback_Topic;
Topic<ChassisFeedback> Chassis_Feedback_Topic;
Topic<ShootFeedback> Shoot_Feedback_Topic;
EventQueue<ShootEvent, 8U> Shoot_Event_Queue;
}

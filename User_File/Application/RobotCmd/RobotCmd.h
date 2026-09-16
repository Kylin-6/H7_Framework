#ifndef ROBOT_CMD_H
#define ROBOT_CMD_H

#include "message_types.h"

bool RobotCmd_RegisterTopics(void);
void RobotCmd_Init(void);
void RobotCmd_Update(void);

void RobotCmd_SetGimbal(const GimbalCmd &command);
void RobotCmd_SetChassis(const ChassisCmd &command);
void RobotCmd_SetShoot(const ShootCmd &command);

bool RobotCmd_GetGimbalFeedback(GimbalFeedback &feedback);
bool RobotCmd_GetChassisFeedback(ChassisFeedback &feedback);
bool RobotCmd_GetShootFeedback(ShootFeedback &feedback);

#endif

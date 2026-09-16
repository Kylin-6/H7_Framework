#include "application.h"

#include "Chassis.h"
#include "Gimbal.h"
#include "RobotCmd.h"
#include "Shoot.h"
#include "dynamic_message_center.h"
#include "message_types.h"
#include <type_traits>

static_assert(std::is_trivially_copyable<GimbalCmd>::value &&
              sizeof(GimbalCmd) <= DYNAMIC_MESSAGE_CENTER_MAX_MESSAGE_SIZE);
static_assert(std::is_trivially_copyable<ChassisCmd>::value &&
              sizeof(ChassisCmd) <= DYNAMIC_MESSAGE_CENTER_MAX_MESSAGE_SIZE);
static_assert(std::is_trivially_copyable<ShootCmd>::value &&
              sizeof(ShootCmd) <= DYNAMIC_MESSAGE_CENTER_MAX_MESSAGE_SIZE);
static_assert(std::is_trivially_copyable<GimbalFeedback>::value &&
              sizeof(GimbalFeedback) <= DYNAMIC_MESSAGE_CENTER_MAX_MESSAGE_SIZE);
static_assert(std::is_trivially_copyable<ChassisFeedback>::value &&
              sizeof(ChassisFeedback) <= DYNAMIC_MESSAGE_CENTER_MAX_MESSAGE_SIZE);
static_assert(std::is_trivially_copyable<ShootFeedback>::value &&
              sizeof(ShootFeedback) <= DYNAMIC_MESSAGE_CENTER_MAX_MESSAGE_SIZE);

extern "C" bool Application_RegisterTopics(void)
{
    return RobotCmd_RegisterTopics() &&
           Gimbal_RegisterTopics() &&
           Chassis_RegisterTopics() &&
           Shoot_RegisterTopics();
}

#include "application.h"

#include "Chassis.h"
#include "Gimbal.h"
#include "RobotCmd.h"
#include "Shoot.h"
#include "dynamic_message_center.h"
#include "message_types.h"
#include <type_traits>

/*
 * 动态通道通过字节队列复制消息。编译期约束可避免把含指针所有权或体积过大的
 * 类型误放入 Message Center，并使错误在固件构建阶段直接暴露。
 */
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
    /* 短路返回：任何端点注册失败都由系统初始化处的 configASSERT 捕获。 */
    return RobotCmd_RegisterTopics() &&
           Gimbal_RegisterTopics() &&
           Chassis_RegisterTopics() &&
           Shoot_RegisterTopics();
}

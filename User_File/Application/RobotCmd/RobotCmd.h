#ifndef ROBOT_CMD_H
#define ROBOT_CMD_H

#include "message_types.h"
#include "output.h"

/** 三个输出均绑定后装载安全启动默认命令；失败时不修改原绑定。 */
bool RobotCmd_Init(Output<GimbalCmd> gimbal_output,
                   Output<ChassisCmd> chassis_output,
                   Output<ShootCmd> shoot_output);
/** 仲裁任务中已解析的输入状态并发布命令；底盘命令每 10 个控制周期刷新。 */
void RobotCmd_Update(void);
/** 设置接口应由 ControlTask 上下文调用；缓存/dirty 标志无同步保护，不可从 ISR/UART 回调并发调用。 */
void RobotCmd_SetGimbal(const GimbalCmd& command);
/** 缓存底盘目标，按每 10 周期刷新；输入许可关闭时拒绝，须与 Update 同一上下文调用。 */
void RobotCmd_SetChassis(const ChassisCmd& command);
/** 缓存发射持续目标并标记发布；输入许可关闭时拒绝，单发动作使用 PushShootEvent。 */
void RobotCmd_SetShoot(const ShootCmd& command);
/** 将一次动作压入容量 8 的 FIFO；输入许可关闭或队列满返回 false，成功不表示已发射。 */
bool RobotCmd_PushShootEvent(const ShootEvent& event);

/** 最近 100 ms 内无对应应用反馈时返回 false，输出对象保持不变；可从任务读取。 */
bool RobotCmd_GetGimbalFeedback(GimbalFeedback& feedback);
/** 读取新鲜底盘反馈；成功后仍需检查 online/ enabled，失败不覆盖输出对象。 */
bool RobotCmd_GetChassisFeedback(ChassisFeedback& feedback);
/** 读取新鲜发射反馈；enabled 不表示达速或射击完成，失败不覆盖输出对象。 */
bool RobotCmd_GetShootFeedback(ShootFeedback& feedback);

#endif

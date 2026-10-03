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
void RobotCmd_SetGimbal(const GimbalCmd &command);
void RobotCmd_SetChassis(const ChassisCmd &command);
void RobotCmd_SetShoot(const ShootCmd &command);
/** 将一次性射击动作压入固定容量 FIFO；队列已满时返回 false。 */
bool RobotCmd_PushShootEvent(const ShootEvent &event);

/** 最近 100 ms 内无对应应用反馈时返回 false，输出对象保持不变；可从任务读取。 */
bool RobotCmd_GetGimbalFeedback(GimbalFeedback &feedback);
bool RobotCmd_GetChassisFeedback(ChassisFeedback &feedback);
bool RobotCmd_GetShootFeedback(ShootFeedback &feedback);

#endif

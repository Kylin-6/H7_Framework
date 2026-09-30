#ifndef RM_REMOTE_INPUT_H
#define RM_REMOTE_INPUT_H

/** 在 RobotCmd_Init 后绑定 UART5 S.BUS，失败时输入互锁保持关闭。 */
bool RemoteInput_Init(void);
/** ControlTask 先调用本函数，再运行 RobotCmd_Update。 */
void RemoteInput_Update(void);

#endif // RM_REMOTE_INPUT_H

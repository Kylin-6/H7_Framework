#ifndef COM_H
#define COM_H

/** 在 RobotCmd_Init 后绑定 UART5 S.BUS，失败时输入互锁保持关闭。 */
bool Communication_Init(void);
/** ControlTask 先调用本函数，再运行 RobotCmd_Update。 */
void Communication_Update(void);

#endif // COM_H

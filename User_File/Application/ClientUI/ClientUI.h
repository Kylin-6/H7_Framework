#ifndef CLIENT_UI_H
#define CLIENT_UI_H

#include "ClientUI_Config.h"

/** StatusTask 启动时调用；Referee 须已由 System_Init 绑定，无硬件操作。
 * robot_id 为 0 时自动获取，否则仅接受红/蓝步兵 3/4/5。失败仅停用 UI。
 */
bool ClientUI_Init(uint16_t robot_id = ClientUIConfig::kRobotId);
/** ControlTask 在机构更新后调用，内部按时间限制为每 10 ms 采集一次。 */
void ClientUI_Capture(void);
/** StatusTask 100 Hz 调用；每次最多提交一包，无延时或动态分配。 */
void ClientUI_Update(void);

#endif

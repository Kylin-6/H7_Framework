#ifndef CHASSIS_H
#define CHASSIS_H

/** 注册底盘命令订阅端和反馈发布端。 */
bool Chassis_RegisterTopics(void);
/** 初始化底盘电机及控制参数。 */
bool Chassis_Init(void);
/** 底盘应用的 1 kHz 周期入口。 */
void Chassis_Update(void);

#endif

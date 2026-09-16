#ifndef SHOOT_H
#define SHOOT_H

/** 注册发射命令订阅端和反馈发布端。 */
bool Shoot_RegisterTopics(void);
/** 初始化摩擦轮、拨弹盘电机及控制参数。 */
bool Shoot_Init(void);
/** 发射应用的 1 kHz 周期入口。 */
void Shoot_Update(void);

#endif

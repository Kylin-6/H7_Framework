#ifndef SHOOT_H
#define SHOOT_H

/** 初始化摩擦轮、拨弹盘电机及控制参数。 */
bool Shoot_Init(void);
/** 发射应用的 1 kHz 周期入口。 */
void Shoot_Update(void);

#endif

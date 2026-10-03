#ifndef CHASSIS_H
#define CHASSIS_H

/** 启动阶段仅调用一次；注册设备与发送组，失败返回 false，不执行机械寻零。 */
bool Chassis_Init(void);
/** 同一 ControlTask 的 1 kHz 入口；消费新鲜速度命令，提交输出，100 Hz 发布反馈。 */
void Chassis_Update(void);

#endif

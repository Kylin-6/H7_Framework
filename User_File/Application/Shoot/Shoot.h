#ifndef SHOOT_H
#define SHOOT_H

/**
 * @brief ControlTask 启动时调用一次，注册三个电机及两个发送组。
 * @return 全部注册成功返回 true；SHOOT=0 时返回 true，不访问硬件。
 */
bool Shoot_Init(void);
/**
 * @brief 每 1 ms 在 RobotCmd_Update 之后调用，消费连续目标与离散事件。
 * @note 反馈每 10 ms 发布；应用未检查命令年龄，上层负责持续刷新安全目标。
 */
void Shoot_Update(void);

#endif

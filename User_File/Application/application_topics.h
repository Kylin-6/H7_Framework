#ifndef APPLICATION_TOPICS_H
#define APPLICATION_TOPICS_H

/*
 * 应用层动态 Topic 名称集中定义，避免发布端和订阅端各自维护字符串。
 * 名称属于通信契约，修改时必须同步考虑所有注册者。
 */
#define APPLICATION_TOPIC_GIMBAL_CMD       "gimbal_cmd"
#define APPLICATION_TOPIC_GIMBAL_FEEDBACK  "gimbal_feedback"
#define APPLICATION_TOPIC_CHASSIS_CMD      "chassis_cmd"
#define APPLICATION_TOPIC_CHASSIS_FEEDBACK "chassis_feedback"
#define APPLICATION_TOPIC_SHOOT_CMD        "shoot_cmd"
#define APPLICATION_TOPIC_SHOOT_FEEDBACK   "shoot_feedback"

#endif

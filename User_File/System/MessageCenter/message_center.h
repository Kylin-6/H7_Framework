#ifndef MESSAGE_CENTER_H
#define MESSAGE_CENTER_H

#include "message_types.h"
#include "topic.h"

/**
 * @brief 系统级静态消息通道。
 * @note 此处只放高频、固定拓扑的状态 Topic；应用命令使用动态消息中心。
 */
namespace MessageCenter
{
/** INS 解算到云台控制的最新姿态状态，实体定义在 message_center.cpp。 */
extern Topic<INS_State> INS_State_Topic;
}

#endif

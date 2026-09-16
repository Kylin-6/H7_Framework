#include "message_center.h"

namespace MessageCenter
{
/* 静态存储期对象：不占用 FreeRTOS 堆，启动后即可直接发布和读取。 */
Topic<INS_State> INS_State_Topic;
}

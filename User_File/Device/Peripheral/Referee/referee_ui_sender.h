#ifndef REFEREE_UI_SENDER_H
#define REFEREE_UI_SENDER_H

#include "referee_protocol_26.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /** 首版 HUD 支持红/蓝步兵 3、4、5；客户端 ID 为 robot_id + 0x100。 */
    uint8_t RefereeUIRobotIdIsValid(uint16_t robot_id);
    /** 已初始化且具有 TX DMA，不表示链路在线或客户端已收到。 */
    uint8_t RefereeUIIsReady(void);
    /** 只读取 CRC/长度合法的 0x0201，age < 500 ms；失败不改输出。 */
    uint8_t RefereeReadUIRobotId(uint16_t* robot_id);

    /** 单一任务调用，提交成功返回 1；无延时/隐式队列，调用方负责限频和重试。
     * 原始图元按现有协议布局编码；count 仅允许 1/2/5/7。
     * 不与旧阻塞 UI 发送器并发使用同一串口。
     */
    uint8_t RefereeUITryDraw(uint16_t robot_id, const interaction_figure_t* figures, uint8_t count);
    uint8_t RefereeUITryText(uint16_t robot_id, const ext_client_custom_character_t* text);
    uint8_t RefereeUITryDeleteLayer(uint16_t robot_id, uint8_t layer);

#ifdef __cplusplus
}
#endif

#endif

#ifndef RM_REFEREE_26_H
#define RM_REFEREE_26_H

#include "usart.h"
#include "referee_protocol_26.h"

extern uint8_t UI_Seq;

#pragma pack(1)
typedef struct
{
    uint8_t Robot_Color;		// 机器人颜色
    uint16_t Robot_ID;			// 本机器人ID
    uint16_t Cilent_ID;			// 本机器人对应的客户端ID
    uint16_t Receiver_Robot_ID; // 机器人车间通信时接收者的ID，必须和本机器人同颜色
} referee_id_t;

// 此结构体包含裁判系统接收数据以及UI绘制与机器人车间通信的相关信息
typedef struct
{
    referee_id_t referee_id;

    xFrameHeader FrameHeader; // 接收到的帧头信息
    uint16_t CmdID;
    game_status_t GameState;							   // 0x0001
    game_result_t GameResult;						   // 0x0002
    game_robot_HP_t GameRobotHP;					   // 0x0003
    event_data_t EventData;							   // 0x0101
    referee_warning_t RefereeWarning;				   // 0x0104
    dart_info_t DartInfo;							   // 0x0105
    robot_status_t GameRobotState;				   // 0x0201
    power_heat_data_t PowerHeatData;				   // 0x0202
    robot_pos_t GameRobotPos;					   // 0x0203
    buff_t BuffMusk;							   // 0x0204
    hurt_data_t RobotHurt;							   // 0x0206
    shoot_data_t ShootData;							   // 0x0207
    projectile_allowance_t ProjectileAllowance;	 // 0x0208
    rfid_status_t RFIDStatus;						   // 0x0209
    dart_client_cmd_t DartClientCmd;				   // 0x020A
    ground_robot_position_t GroundRobotPosition; // 0x020B
    radar_mark_data_t RadarMarkData;				   // 0x020C
    sentry_info_t SentryInfo;						   // 0x020D
    radar_info_t RadarInfo;						   // 0x020E
    
    uint8_t init_flag;

} referee_info_t;

#pragma pack()

/**
 * @brief 向 UART BSP 注册裁判数据接收回调；当前 System_Init 不自动调用。
 * @param referee_usart_handle 已配置 RX DMA 的 UART 句柄，由板级接线选择。
 * @return 静态反馈对象指针；空句柄返回 NULL。
 * @note 回调在 UART 中断中运行有界流式解析；跨回调保留半帧，已知命令校验固定载荷长度。
 */
referee_info_t *RefereeInit(UART_HandleTypeDef *referee_usart_handle);

/**
 * @brief 提交 UI/交互数据；只有 UART 提交成功才阻塞延时 115 ms。
 * @param send 发送缓冲首地址；BSP 的 DMA 发送路径会复制内容。
 * @param tx_len 字节数。
 * @note 无返回值，不能用于 ISR 或 1 kHz 控制任务；调用前须先 RefereeInit。
 */
void RefereeSend(uint8_t *send, uint16_t tx_len);

/** @brief 解析本次 DMA chunk，不注册 UART，与接收回调共用跨调用半帧缓冲。 */
void RefereeReceiveData(uint8_t *data, uint16_t length);
uint8_t RefereeIsEnabled(void);   ///< 已完成 UART 注册；裁判系统没有协议使能态。
uint8_t RefereeIsOnline(void);    ///< 最近 500 ms 内解析到过 CRC 合法帧。
uint8_t RefereeIsDataValid(void); ///< 当前等价于 Online。
uint8_t RefereeIsHealthy(void);   ///< 已初始化且当前数据在线。

#endif // !REFEREE_26_H

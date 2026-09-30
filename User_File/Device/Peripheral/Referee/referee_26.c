/**
 * @file rm_referee.C
 * @author kidneygood (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-11-18
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "referee_26.h"
#include "string.h"
#include "crc_ref.h"
#include "bsp_uart.h"
#include "cmsis_os.h"
#include "referee_daemon.h"

#define RE_RX_BUFFER_SIZE 320u // 覆盖当前协议最长 300 字节数据段及 9 字节包头/尾

static UART_HandleTypeDef *referee_uart; // 裁判系统串口实例
static referee_info_t referee_info;			  // 裁判系统数据
static uint8_t referee_rx_buffer[RE_RX_BUFFER_SIZE];
static uint16_t referee_rx_length;

/* 已校验 CRC 的已知命令必须匹配该命令的固定载荷长度；未知命令只计链路在线。 */
static uint8_t JudgeStoreFrame(const uint8_t *frame, uint16_t payload_length)
{
    const uint16_t command_id = (uint16_t)frame[CMD_ID_Offset] |
                                ((uint16_t)frame[CMD_ID_Offset + 1U] << 8U);
#define JUDGE_COPY(cmd, field, expected) \
    case cmd: \
        if (payload_length != (expected) || (expected) != sizeof(referee_info.field)) return FALSE; \
        memcpy(&referee_info.field, frame + DATA_Offset, (expected)); \
        break
    switch (command_id)
    {
        JUDGE_COPY(ID_game_status, GameState, LEN_game_status);
        JUDGE_COPY(ID_game_result, GameResult, LEN_game_result);
        JUDGE_COPY(ID_game_robot_HP, GameRobotHP, LEN_game_robot_HP);
        JUDGE_COPY(ID_event_data, EventData, LEN_event_data);
        JUDGE_COPY(ID_referee_warning, RefereeWarning, LEN_referee_warning);
        JUDGE_COPY(ID_dart_info, DartInfo, LEN_dart_info);
        JUDGE_COPY(ID_robot_status, GameRobotState, LEN_robot_status);
        JUDGE_COPY(ID_power_heat_data, PowerHeatData, LEN_power_heat_data);
        JUDGE_COPY(ID_robot_pos, GameRobotPos, LEN_robot_pos);
        JUDGE_COPY(ID_buff, BuffMusk, LEN_buff);
        JUDGE_COPY(ID_hurt_data, RobotHurt, LEN_hurt_data);
        JUDGE_COPY(ID_shoot_data, ShootData, LEN_shoot_data);
        JUDGE_COPY(ID_projectile_allowance, ProjectileAllowance, LEN_projectile_allowance);
        JUDGE_COPY(ID_rfid_status, RFIDStatus, LEN_rfid_status);
        JUDGE_COPY(ID_dart_client_cmd, DartClientCmd, LEN_dart_client_cmd);
        JUDGE_COPY(ID_ground_robot_position, GroundRobotPosition, LEN_ground_robot_position);
        JUDGE_COPY(ID_radar_mark_data, RadarMarkData, LEN_radar_mark_data);
        JUDGE_COPY(ID_sentry_info, SentryInfo, LEN_sentry_info);
        JUDGE_COPY(ID_radar_info, RadarInfo, LEN_radar_info);
    default:
        break;
    }
#undef JUDGE_COPY
    memcpy(&referee_info.FrameHeader, frame, LEN_HEADER);
    referee_info.CmdID = command_id;
    referee_info.init_flag = 1U;
    if (referee_uart != NULL) { RefereeDaemonFeed(); }
    return TRUE;
}

static void JudgeDropPrefix(uint16_t count)
{
    referee_rx_length -= count;
    memmove(referee_rx_buffer, referee_rx_buffer + count, referee_rx_length);
}

/** @brief 有界流式解析；UART ISR 是唯一写入者，保留跨 DMA 回调半帧。 */
static void JudgeReadData(uint8_t *buff, uint16_t length)
{
    if (buff == NULL)
        return;

    for (uint16_t index = 0U; index < length; ++index)
    {
        if (referee_rx_length == 0U && buff[index] != REFEREE_SOF)
            continue;
        if (referee_rx_length == RE_RX_BUFFER_SIZE)
            JudgeDropPrefix(1U);
        referee_rx_buffer[referee_rx_length++] = buff[index];

        while (referee_rx_length >= LEN_HEADER)
        {
            if (referee_rx_buffer[0] != REFEREE_SOF ||
                Verify_CRC8_Check_Sum(referee_rx_buffer, LEN_HEADER) != TRUE)
            {
                JudgeDropPrefix(1U);
                continue;
            }

            const uint32_t payload_length = (uint32_t)referee_rx_buffer[DATA_LENGTH] |
                                            ((uint32_t)referee_rx_buffer[DATA_LENGTH + 1U] << 8U);
            const uint32_t frame_length = payload_length + LEN_HEADER + LEN_CMDID + LEN_TAIL;
            if (frame_length > RE_RX_BUFFER_SIZE)
            {
                JudgeDropPrefix(1U);
                continue;
            }
            if (referee_rx_length < frame_length)
                break;
            if (Verify_CRC16_Check_Sum(referee_rx_buffer, frame_length) == TRUE)
            {
                (void)JudgeStoreFrame(referee_rx_buffer, (uint16_t)payload_length);
                JudgeDropPrefix((uint16_t)frame_length);
            }
            else
            {
                /* CRC 错帧按一字节滑动，仍有机会找到嵌在其中的下一个帧头。 */
                JudgeDropPrefix(1U);
            }
        }
    }
}

/* UART BSP 交付的是本次 DMA chunk，不保证恰好一帧。 */
static void RefereeRxCallback(uint8_t *buffer, uint16_t length)
{
    JudgeReadData(buffer, length);
}

/* 裁判系统通信初始化 */
referee_info_t *RefereeInit(UART_HandleTypeDef *referee_usart_handle)
{
    if (referee_usart_handle == NULL)
        return NULL;

    if (!RefereeDaemonRegister())
    {
        referee_uart = NULL;
        return NULL;
    }
    memset(&referee_info, 0, sizeof(referee_info));
    referee_rx_length = 0U;
    referee_uart = referee_usart_handle;
    UART_Init(referee_usart_handle, RefereeRxCallback);

    return &referee_info;
}

/** 提交成功后在调用任务阻塞 115 ms；不适用于中断或 1 kHz 控制循环。 */
void RefereeSend(uint8_t *send, uint16_t tx_len)
{
    if (referee_uart != NULL &&
        UART_Transmit_Data(referee_uart, send, tx_len) == HAL_OK)
        osDelay(115);
}

void RefereeReceiveData(uint8_t *data, uint16_t length)
{
    JudgeReadData(data, length);
}

uint8_t RefereeIsEnabled(void) { return referee_uart != NULL; }

uint8_t RefereeIsOnline(void)
{
    return RefereeIsEnabled() && referee_info.init_flag &&
           RefereeDaemonIsOnline();
}

uint8_t RefereeIsDataValid(void) { return RefereeIsOnline(); }
uint8_t RefereeIsHealthy(void) { return RefereeIsEnabled() && RefereeIsDataValid(); }

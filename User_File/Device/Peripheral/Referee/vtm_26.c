#include "vtm_26.h"
#include "cmsis_os.h"
#include <string.h>
#include "bsp_uart.h"
#include "crc_ref.h"
#include "referee_daemon.h"

#define VTM_RX_BUFFER_SIZE 255u // 图传接收缓冲区大小
#define RC_FRAME_LEN 21u        // 遥控数据帧固定长度(字节)

static UART_HandleTypeDef *vtm_uart; // 图传串口实例
static vtm_info_t vtm_info;						// 图传数据
static uint8_t vtm_has_valid_frame;

/**
 * @brief  读取图传数据,中断中读取保证速度
 * @param  buff: 读取到的图传原始数据
 * @attention  缓冲区中可能包含多帧数据(遥控帧0xA9/图传链路帧0xA5混合),
 *             使用循环逐帧解析,避免递归导致中断上下文栈溢出
 */
static void VTMReadData(uint8_t *buff, uint16_t length)
{
    uint16_t vtm_length;       // 统计一帧数据长度
    uint16_t read_offset = 0;  // 记录相对于buff起始地址的累计偏移量,用于越界保护
    if (buff == NULL)	// 空数据包，则不作任何处理
        return;

    // 使用循环逐帧解析缓冲区中的所有数据
    // 至少需要2字节来判断帧头类型(0xA9 0x53 或 0xA5)
    while (read_offset + 2 <= length)
    {
        uint8_t *frame = buff + read_offset; // 当前帧起始地址

        // 判断帧头: 0xA9 0x53 为VTM遥控数据
        if (frame[0] == RC_SOF1 && frame[1] == RC_SOF2)
        {
            // 越界保护: 遥控帧完整长度不能超出缓冲区剩余空间
            if (read_offset + RC_FRAME_LEN > length)
                break;
            // 帧尾CRC16校验
            if (Verify_CRC16_Check_Sum(frame, RC_FRAME_LEN) == TRUE)
            {
                memcpy(&vtm_info.rc_ctrl, frame, sizeof(RC_ctrl_t));
                vtm_has_valid_frame = 1U;
                if (vtm_uart != NULL) { VTMDaemonFeed(); }
            }
            // 遥控帧固定21字节,偏移量前移至下一帧
            read_offset += RC_FRAME_LEN;
        }
        // 判断帧头: 0xA5 为图传链路数据,按照裁判系统协议处理
        else if (frame[0] == REFEREE_SOF)
        {
            // 确保剩余空间至少能容纳一个帧头(5字节)
            if (read_offset + LEN_HEADER > length)
                break;

            if (Verify_CRC8_Check_Sum(frame, LEN_HEADER) != TRUE)
            {
                read_offset++;
                continue;
            }
            // 先用宽整数检查完整长度，避免声明长度溢出后误把短帧当作合法数据。
            const uint32_t frame_length = (uint32_t)frame[DATA_LENGTH] |
                                          ((uint32_t)frame[DATA_LENGTH + 1U] << 8U);
            const uint32_t total_length = frame_length + LEN_HEADER + LEN_CMDID + LEN_TAIL;
            if (total_length > (uint32_t)(length - read_offset))
                break;
            vtm_length = (uint16_t)total_length;
            if (Verify_CRC16_Check_Sum(frame, vtm_length) == TRUE)
            {
                memcpy(&vtm_info.FrameHeader, frame, LEN_HEADER);
                vtm_info.CmdID = (frame[6] << 8 | frame[5]);
                vtm_has_valid_frame = 1U;
                if (vtm_uart != NULL) { VTMDaemonFeed(); }
                // TODO: 图传链路业务载荷解析(未实现)
            }
            read_offset += vtm_length;
        }
        else
        {
            // 跳过噪声，继续搜索下一帧。
            read_offset++;
        }
    }
}

static void VTMRxCallback(uint8_t *buffer, uint16_t length)
{
    VTMReadData(buffer, length);
}

vtm_info_t *VTMInit(UART_HandleTypeDef *vtm_usart_handle)
{
    if (vtm_usart_handle == NULL)
        return NULL;

    if (!VTMDaemonRegister())
    {
        vtm_uart = NULL;
        return NULL;
    }
    memset(&vtm_info, 0, sizeof(vtm_info));
    vtm_has_valid_frame = 0U;
    vtm_uart = vtm_usart_handle;
    UART_Init(vtm_usart_handle, VTMRxCallback);

    // vtm_info默认为全0,上电如果没连图传串口线会导致底盘乱跑
    // 所以初始化为摇杆归中
    vtm_info.rc_ctrl.rc.bit.stick_RH = 1024;
    vtm_info.rc_ctrl.rc.bit.stick_RV = 1024;
    vtm_info.rc_ctrl.rc.bit.stick_LV = 1024;
    vtm_info.rc_ctrl.rc.bit.stick_LH = 1024;
    vtm_info.rc_ctrl.rc.bit.dial = 1024;

    return &vtm_info;
}

void VTMSend(uint8_t *send, uint16_t tx_len)
{
    if (vtm_uart != NULL && UART_Transmit_Data(vtm_uart, send, tx_len) == HAL_OK)
        osDelay(115);
}

void VTMReceiveData(uint8_t *data, uint16_t length)
{
    VTMReadData(data, length);
}

uint8_t VTMIsEnabled(void) { return vtm_uart != NULL; }
uint8_t VTMIsOnline(void)
{
    return VTMIsEnabled() && vtm_has_valid_frame && VTMDaemonIsOnline();
}
uint8_t VTMIsDataValid(void) { return VTMIsOnline(); }
uint8_t VTMIsHealthy(void) { return VTMIsEnabled() && VTMIsDataValid(); }

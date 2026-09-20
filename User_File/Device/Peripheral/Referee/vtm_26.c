#include "vtm_26.h"
#include "cmsis_os.h"
#include <string.h>
#include "bsp_uart.h"
#include "crc_ref.h"

#define VTM_RX_BUFFER_SIZE 255u // 图传接收缓冲区大小
#define RC_FRAME_LEN 21u        // 遥控数据帧固定长度(字节)

static UART_HandleTypeDef *vtm_uart; // 图传串口实例
static vtm_info_t vtm_info;						// 图传数据

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

            memcpy(&vtm_info.FrameHeader, frame, LEN_HEADER);

            if (Verify_CRC8_Check_Sum(frame, LEN_HEADER) == TRUE)
            {
                vtm_length = (uint16_t)(vtm_info.FrameHeader.DataLength + LEN_HEADER + LEN_CMDID + LEN_TAIL);

                // 越界保护: 当前帧完整长度不能超出缓冲区剩余空间
                if (read_offset + vtm_length > length)
                    break;

                if (Verify_CRC16_Check_Sum(frame, vtm_length) == TRUE)
                {
                    vtm_info.CmdID = (frame[6] << 8 | frame[5]);
                    // TODO: 图传链路数据解析(未实现)
                }
            }

            // 按帧头中声明的长度跳过当前帧,无论校验是否通过
            uint16_t frame_len = (uint16_t)(sizeof(xFrameHeader) + LEN_CMDID + vtm_info.FrameHeader.DataLength + LEN_TAIL);
            read_offset += frame_len;
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

    memset(&vtm_info, 0, sizeof(vtm_info));
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

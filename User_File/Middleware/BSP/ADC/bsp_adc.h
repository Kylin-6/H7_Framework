/**
 * @file bsp_adc.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief 仿照SCUT-Robotlab改写的ADC初始化与配置流程
 * @version 1.1
 * @date 2023-08-29 0.1 23赛季定稿
 * @date 2025-08-13 1.1 适配达妙MC02板
 *
 * @copyright USTC-RoboWalker (c) 2023-2025
 *
 */

#ifndef DRV_ADC_H
#define DRV_ADC_H

/* Includes ------------------------------------------------------------------*/

#include "adc.h"
#include "stm32h7xx_hal.h"
#include <stdbool.h>

/* Exported macros -----------------------------------------------------------*/

// uint16_t 采样元素个数（每个管理对象占用 256 字节的采样数组）
#define ADC_BUFFER_SIZE 128

/* Exported types ------------------------------------------------------------*/

/**
 * @brief ADC采样信息结构体
 *
 */
struct Struct_ADC_Manage_Object
{
    ADC_HandleTypeDef *ADC_Handler;
    uint16_t ADC_Data[ADC_BUFFER_SIZE];
};

/* Exported variables --------------------------------------------------------*/

extern struct Struct_ADC_Manage_Object ADC1_Manage_Object;
extern struct Struct_ADC_Manage_Object ADC2_Manage_Object;
extern struct Struct_ADC_Manage_Object ADC3_Manage_Object;

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 校准 ADC 并启动指定采样元素数的 DMA 接收；循环模式由 CubeMX 配置决定。
 * @param Sample_Number DMA 传输元素数，不是字节数；调用方须保证不超过 ADC_BUFFER_SIZE。
 * @return 参数、校准、DMA 启动任一失败时返回 false；调用方可据此降级启动。
 */
bool ADC_Init(ADC_HandleTypeDef *hadc, uint16_t Sample_Number);

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/

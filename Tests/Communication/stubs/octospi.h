#pragma once
#include "stm32h7xx_hal.h"
enum { OCTOSPI1 = 1, OCTOSPI2, HAL_OSPI_TIMEOUT_DEFAULT_VALUE = 5000 };
enum { HAL_OSPI_DATA_NONE = 0, HAL_OSPI_DATA_1_LINE = 1 };
struct OSPI_HandleTypeDef { int Instance; };
struct OSPI_RegularCmdTypeDef { uint32_t Instruction, NbData, DataMode; };
struct OSPI_AutoPollingTypeDef { uint32_t Match; };
HAL_StatusTypeDef HAL_OSPI_Command(OSPI_HandleTypeDef *, OSPI_RegularCmdTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_OSPI_Transmit_DMA(OSPI_HandleTypeDef *, uint8_t *);
HAL_StatusTypeDef HAL_OSPI_Receive_DMA(OSPI_HandleTypeDef *, uint8_t *);
HAL_StatusTypeDef HAL_OSPI_AutoPolling_IT(OSPI_HandleTypeDef *, OSPI_AutoPollingTypeDef *);

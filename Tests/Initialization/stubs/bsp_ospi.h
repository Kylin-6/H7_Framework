#pragma once
#include "stm32h7xx_hal.h"
#include "sys_timestamp.h"
#define OSPI_BUFFER_SIZE 512
enum {
    HAL_OSPI_STATE_READY = 1, HAL_OSPI_FLAG_SM = 1, HAL_OSPI_FLAG_TC = 2, HAL_OSPI_FLAG_TE = 4,
    HAL_OSPI_OPTYPE_COMMON_CFG = 0, HAL_OSPI_FLASH_ID_1 = 0,
    HAL_OSPI_INSTRUCTION_1_LINE = 1, HAL_OSPI_INSTRUCTION_8_BITS = 8,
    HAL_OSPI_INSTRUCTION_DTR_DISABLE = 0, HAL_OSPI_ADDRESS_NONE = 0,
    HAL_OSPI_ADDRESS_1_LINE = 1, HAL_OSPI_ADDRESS_4_LINES = 4,
    HAL_OSPI_ADDRESS_24_BITS = 24, HAL_OSPI_ADDRESS_DTR_DISABLE = 0,
    HAL_OSPI_ALTERNATE_BYTES_NONE = 0, HAL_OSPI_ALTERNATE_BYTES_DTR_DISABLE = 0,
    HAL_OSPI_DATA_NONE = 0, HAL_OSPI_DATA_1_LINE = 1, HAL_OSPI_DATA_4_LINES = 4,
    HAL_OSPI_DATA_DTR_DISABLE = 0, HAL_OSPI_DQS_DISABLE = 0,
    HAL_OSPI_SIOO_INST_EVERY_CMD = 0, HAL_OSPI_MATCH_MODE_AND = 0,
    HAL_OSPI_AUTOMATIC_STOP_ENABLE = 1
};
#define __HAL_OSPI_CLEAR_FLAG(handler, flags) ((void)0)
struct OSPI_HandleTypeDef { unsigned State; };
struct OSPI_RegularCmdTypeDef {
    uint32_t OperationType, FlashId, Instruction, InstructionMode, InstructionSize,
        InstructionDtrMode, Address, AddressMode, AddressSize, AddressDtrMode,
        AlternateBytes, AlternateBytesMode, AlternateBytesSize, AlternateBytesDtrMode,
        DataMode, NbData, DataDtrMode, DummyCycles, DQSMode, SIOOMode;
};
struct OSPI_AutoPollingTypeDef { uint32_t Match, Mask, MatchMode, AutomaticStop, Interval; };
struct OSPI_MemoryMappedTypeDef { uint32_t TimeOutActivation; };
struct Struct_OSPI_Manage_Object {
    OSPI_HandleTypeDef *OSPI_Handler;
    uint8_t Tx_Buffer[OSPI_BUFFER_SIZE], Rx_Buffer[OSPI_BUFFER_SIZE];
    uint64_t Auto_Polling_Timestamp;
};
extern Struct_OSPI_Manage_Object OSPI2_Manage_Object;
HAL_StatusTypeDef OSPI_Command_Receive_Data(OSPI_HandleTypeDef *, OSPI_RegularCmdTypeDef *);
HAL_StatusTypeDef OSPI_Command_Transmit_Data(OSPI_HandleTypeDef *, OSPI_RegularCmdTypeDef *);
HAL_StatusTypeDef OSPI_Command(OSPI_HandleTypeDef *, OSPI_RegularCmdTypeDef *);
HAL_StatusTypeDef OSPI_Auto_Polling(OSPI_HandleTypeDef *, OSPI_AutoPollingTypeDef *);
HAL_StatusTypeDef HAL_OSPI_MemoryMapped(OSPI_HandleTypeDef *, OSPI_MemoryMappedTypeDef *);

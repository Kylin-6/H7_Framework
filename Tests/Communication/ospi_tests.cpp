#include "bsp_ospi.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(value) do { if (!(value)) { fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #value); exit(1); } } while (0)
Class_Timestamp SYS_Timestamp;
static HAL_StatusTypeDef command_status = HAL_OK;
static HAL_StatusTypeDef dma_status = HAL_OK;
static uint32_t commands, tx, rx;
static uint8_t *dma_buffer;
HAL_StatusTypeDef HAL_OSPI_Command(OSPI_HandleTypeDef *, OSPI_RegularCmdTypeDef *, uint32_t) { ++commands; return command_status; }
HAL_StatusTypeDef HAL_OSPI_Transmit_DMA(OSPI_HandleTypeDef *, uint8_t *data) { ++tx; dma_buffer = data; return dma_status; }
HAL_StatusTypeDef HAL_OSPI_Receive_DMA(OSPI_HandleTypeDef *, uint8_t *data) { ++rx; dma_buffer = data; return dma_status; }
HAL_StatusTypeDef HAL_OSPI_AutoPolling_IT(OSPI_HandleTypeDef *, OSPI_AutoPollingTypeDef *) { return HAL_OK; }
int main(int argc, char **argv)
{
    CHECK(argc == 2);
    OSPI_HandleTypeDef bus1 = {OCTOSPI1}, bus2 = {OCTOSPI2};
    OSPI_Init(&bus1, nullptr, nullptr, nullptr);
    OSPI_Init(&bus2, nullptr, nullptr, nullptr);
    OSPI_RegularCmdTypeDef command = {0x9f, 3, HAL_OSPI_DATA_1_LINE};
    if (!strcmp(argv[1], "normal"))
    {
        OSPI_Command_Receive_Data(&bus1, &command);
        CHECK(commands == 1 && rx == 1 && tx == 0 && dma_buffer == OSPI1_Manage_Object.Rx_Buffer);
        OSPI_Command_Transmit_Data(&bus2, &command);
        CHECK(commands == 2 && rx == 1 && tx == 1 && dma_buffer == OSPI2_Manage_Object.Tx_Buffer);
    }
    else if (!strcmp(argv[1], "command_failure"))
    {
        const HAL_StatusTypeDef failures[] = {HAL_ERROR, HAL_BUSY, HAL_TIMEOUT};
        for (unsigned index = 0; index < sizeof(failures) / sizeof(failures[0]); ++index)
        {
            HAL_StatusTypeDef failure = failures[index];
            command_status = failure;
            CHECK(OSPI_Command_Receive_Data(&bus1, &command) == failure);
            CHECK(OSPI_Command_Transmit_Data(&bus2, &command) == failure);
            CHECK(rx == 0 && tx == 0);
        }
        CHECK(commands == 6);
    }
    else if (!strcmp(argv[1], "bounds"))
    {
        command.NbData = OSPI_BUFFER_SIZE + 1;
        OSPI_Command_Receive_Data(&bus1, &command);
        OSPI_Command_Transmit_Data(&bus2, &command);
        CHECK(commands == 0 && rx == 0 && tx == 0);
        command.NbData = 0;
        OSPI_Command_Receive_Data(&bus1, &command);
        CHECK(commands == 0 && rx == 0);
        command.NbData = OSPI_BUFFER_SIZE;
        OSPI_Command_Receive_Data(&bus1, &command);
        CHECK(commands == 1 && rx == 1);
    }
    else if (!strcmp(argv[1], "dma_failure"))
    {
        const HAL_StatusTypeDef failures[] = {HAL_ERROR, HAL_BUSY, HAL_TIMEOUT};
        for (unsigned index = 0; index < sizeof(failures) / sizeof(failures[0]); ++index)
        {
            HAL_StatusTypeDef failure = failures[index];
            dma_status = failure;
            CHECK(OSPI_Command_Receive_Data(&bus1, &command) == failure);
            CHECK(OSPI_Command_Transmit_Data(&bus2, &command) == failure);
        }
        CHECK(commands == 6 && rx == 3 && tx == 3);
    }
    else if (!strcmp(argv[1], "unsupported"))
    {
        OSPI_HandleTypeDef unknown = {99};
        CHECK(OSPI_Command_Transmit_Receive_Data(&bus1, &command) == HAL_ERROR);
        CHECK(OSPI_Command_Receive_Data(nullptr, &command) == HAL_ERROR);
        CHECK(OSPI_Command_Transmit_Data(&unknown, &command) == HAL_ERROR);
        CHECK(OSPI_Command(&bus1, nullptr) == HAL_ERROR);
        command.DataMode = HAL_OSPI_DATA_NONE;
        CHECK(OSPI_Command_Receive_Data(&bus1, &command) == HAL_ERROR);
        CHECK(commands == 0 && rx == 0 && tx == 0);
    }
    else { CHECK(false); }
    printf("PASS OSPI %s\n", argv[1]);
}

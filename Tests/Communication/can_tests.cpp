#include "bsp_can.h"
#include "cmsis_os2.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(value) do { if (!(value)) { fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #value); exit(1); } } while (0)
FDCAN_HandleTypeDef hfdcan1 = {1}, hfdcan2 = {2}, hfdcan3 = {3};
static uint32_t remote_std, remote_ext, configured, pending, deliveries, primask;
static FDCAN_RxHeaderTypeDef incoming;
extern "C" uint32_t __get_PRIMASK() { return primask; }
extern "C" void __disable_irq() { primask = 1; }
extern "C" void __enable_irq() { primask = 0; }
extern "C" void __set_PRIMASK(uint32_t value) { primask = value; }
extern "C" void __DMB() {}
extern "C" void Error_Handler() { CHECK(false); }
extern "C" osMessageQueueId_t osMessageQueueNew(uint32_t, uint32_t, const void *) { return &incoming; }
extern "C" osStatus_t osMessageQueuePut(osMessageQueueId_t, const void *, uint8_t, uint32_t) { return osOK; }
extern "C" osStatus_t osMessageQueueGet(osMessageQueueId_t, void *, uint8_t *, uint32_t) { return osErrorResource; }
extern "C" HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *, FDCAN_FilterTypeDef *) { return HAL_OK; }
extern "C" HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *, uint32_t, uint32_t, uint32_t a, uint32_t b)
{ remote_std = a; remote_ext = b; ++configured; return HAL_OK; }
extern "C" HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *, uint32_t, uint32_t) { return HAL_OK; }
extern "C" HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *) { return HAL_OK; }
extern "C" uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef *, uint32_t) { return pending; }
extern "C" HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *, uint32_t, FDCAN_RxHeaderTypeDef *header, uint8_t *data)
{
    // Same copy length as this repository's STM32H7 HAL, including classic DLC 9..15.
    static const uint8_t bytes[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
    *header = incoming;
    memset(data, 0xa5, bytes[incoming.DataLength]);
    --pending;
    return HAL_OK;
}
extern "C" uint32_t HAL_FDCAN_GetTxFifoFreeLevel(FDCAN_HandleTypeDef *) { return 1; }
extern "C" HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *, const FDCAN_TxHeaderTypeDef *, const uint8_t *) { return HAL_OK; }
static void Receive(FDCAN_HandleTypeDef *bus, uint32_t id, uint8_t *data, uint32_t length, void *context)
{ CHECK(bus == &hfdcan1 && id == 0x201 && data[0] == 0xa5 && length == 8 && context == &incoming); ++deliveries; }
int main(int argc, char **argv)
{
    CHECK(argc == 2);
    BSP_CAN_ConfigInit();
    CHECK(BSP_CAN_RegisterCallback(0x201, &hfdcan1, Receive, &incoming));
    incoming.Identifier = 0x201;
    incoming.DataLength = FDCAN_DLC_BYTES_8;
    pending = 1;
    if (!strcmp(argv[1], "data"))
    {
        HAL_FDCAN_RxFifo0Callback(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
        CHECK(deliveries == 1 && pending == 0);
        pending = 1;
        HAL_FDCAN_RxFifo0Callback(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
        CHECK(deliveries == 1 && pending == 0);
    }
    else if (!strcmp(argv[1], "remote"))
    {
        incoming.RxFrameType = FDCAN_REMOTE_FRAME;
        HAL_FDCAN_RxFifo0Callback(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
        CHECK(deliveries == 0 && pending == 0);
    }
    else if (!strcmp(argv[1], "filter"))
    { CHECK(configured == 3 && remote_std == FDCAN_REJECT_REMOTE && remote_ext == FDCAN_REJECT_REMOTE); }
    else if (!strcmp(argv[1], "classic_dlc"))
    {
        for (uint32_t dlc = 9; dlc <= 15; ++dlc)
        {
            incoming.DataLength = dlc;
            pending = 1;
            HAL_FDCAN_RxFifo0Callback(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
            CHECK(deliveries == dlc - 8 && pending == 0);
        }
    }
    else if (!strcmp(argv[1], "storage"))
    {
        incoming.Identifier = 0x202; // no callback; let the compiler's stack guard check the return
        incoming.DataLength = 15;
        HAL_FDCAN_RxFifo0Callback(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
        CHECK(deliveries == 0 && pending == 0);
    }
    else if (!strcmp(argv[1], "unsupported"))
    {
        incoming.IdType = FDCAN_EXTENDED_ID;
        HAL_FDCAN_RxFifo0Callback(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
        CHECK(deliveries == 0 && pending == 0);
        pending = 1;
        incoming.IdType = FDCAN_STANDARD_ID;
        incoming.FDFormat = FDCAN_FD_CAN;
        HAL_FDCAN_RxFifo0Callback(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
        CHECK(deliveries == 0 && pending == 0);
    }
    else { CHECK(false); }
    printf("PASS CAN %s\n", argv[1]);
}

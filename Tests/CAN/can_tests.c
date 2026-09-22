#include "bsp_can.h"
#include "cmsis_os2.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

FDCAN_HandleTypeDef hfdcan1 = {1}, hfdcan2 = {2}, hfdcan3 = {3};
uint32_t test_primask;
static int queue_put_fails;
static int queue_has_message;
static Struct_CAN_Tx_Msg queued_message;
static uint32_t fifo_free = 1U;
static HAL_StatusTypeDef add_status = HAL_OK;

static unsigned checks;
#define CHECK(condition) do { ++checks; if (!(condition)) { \
    fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition); exit(1); } } while (0)

osMessageQueueId_t osMessageQueueNew(uint32_t count, uint32_t size, const void *attr)
{ (void)count; (void)size; (void)attr; return &queued_message; }
osStatus_t osMessageQueuePut(osMessageQueueId_t queue, const void *message, uint8_t priority, uint32_t timeout)
{ (void)queue; (void)priority; (void)timeout; if (queue_put_fails) return -1; queued_message = *(const Struct_CAN_Tx_Msg *)message; queue_has_message = 1; return osOK; }
osStatus_t osMessageQueueGet(osMessageQueueId_t queue, void *message, uint8_t *priority, uint32_t timeout)
{ (void)queue; (void)priority; (void)timeout; if (!queue_has_message) return -1; *(Struct_CAN_Tx_Msg *)message = queued_message; queue_has_message = 0; return osOK; }

HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *h, FDCAN_FilterTypeDef *f) { (void)h; (void)f; return HAL_OK; }
HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *h, uint32_t a, uint32_t b, uint32_t c, uint32_t d) { (void)h; (void)a; (void)b; (void)c; (void)d; return HAL_OK; }
HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *h, uint32_t a, uint32_t b) { (void)h; (void)a; (void)b; return HAL_OK; }
HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *h) { (void)h; return HAL_OK; }
uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef *h, uint32_t f) { (void)h; (void)f; return 0; }
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *h, uint32_t f, FDCAN_RxHeaderTypeDef *r, uint8_t *d) { (void)h; (void)f; (void)r; (void)d; return HAL_OK; }
uint32_t HAL_FDCAN_GetTxFifoFreeLevel(FDCAN_HandleTypeDef *h) { (void)h; return fifo_free; }
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *h, FDCAN_TxHeaderTypeDef *t, const uint8_t *d) { (void)h; (void)t; (void)d; return add_status; }
void Error_Handler(void) { abort(); }

int main(void)
{
    Struct_CAN_Tx_Stats stats;
    Struct_CAN_Tx_Msg message = {&hfdcan1, 1U, {0}, 8U};
    BSP_CAN_ConfigInit();
    BSP_CAN_GetTxStats(&stats);
    CHECK(stats.submit_queue_full_count == 0U);

    queue_put_fails = 1;
    CHECK(!CAN_Tx_Submit(&message));
    BSP_CAN_GetTxStats(&stats);
    CHECK(stats.submit_queue_full_count == 1U);
    queue_put_fails = 0;

    for (uint32_t id = 0U; id < 32U; ++id) { message.id = id; CHECK(CAN_Tx_Perform(&message)); }
    message.id = 32U;
    CHECK(!CAN_Tx_Perform(&message));
    BSP_CAN_GetTxStats(&stats);
    CHECK(stats.periodic_slot_full_count == 1U);

    BSP_CAN_ConfigInit();
    message.id = 1U;
    CHECK(CAN_Tx_Perform(&message));
    fifo_free = 0U;
    CHECK(!BSP_CAN_SendPer());
    BSP_CAN_GetTxStats(&stats);
    CHECK(stats.hardware_fifo_full_count == 1U);

    fifo_free = 1U;
    add_status = -1;
    CHECK(!BSP_CAN_SendPer());
    BSP_CAN_GetTxStats(&stats);
    CHECK(stats.hal_send_error_count == 1U);

    BSP_CAN_ConfigInit();
    BSP_CAN_GetTxStats(&stats);
    CHECK(stats.submit_queue_full_count == 0U && stats.periodic_slot_full_count == 0U &&
          stats.hardware_fifo_full_count == 0U && stats.hal_send_error_count == 0U);
    printf("PASS CAN stats: %u checks\n", checks);
    return 0;
}

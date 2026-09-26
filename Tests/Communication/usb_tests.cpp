#include "bsp_usb.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(value) do { if (!(value)) { fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #value); exit(1); } } while (0)
Class_Timestamp SYS_Timestamp;
USBD_HandleTypeDef hUsbDeviceHS;
volatile bool init_finished = true;
static USBD_CDC_HandleTypeDef cdc;
static uint32_t primask, received, rx_rearms, tx_starts;
static uint8_t delivered[USB_BUFFER_SIZE];
static uint8_t *armed_rx;
extern uint8_t UserRxBufferHS[APP_RX_DATA_SIZE];

extern "C" uint32_t __get_PRIMASK() { return primask; }
extern "C" void __disable_irq() { primask = 1; }
extern "C" void __set_PRIMASK(uint32_t value) { primask = value; }
extern "C" void __DMB() {}
extern "C" uint8_t USBD_CDC_SetRxBuffer(USBD_HandleTypeDef *, uint8_t *data) { cdc.RxBuffer = data; return USBD_OK; }
extern "C" uint8_t USBD_CDC_SetTxBuffer(USBD_HandleTypeDef *, uint8_t *data, uint32_t length) { cdc.TxBuffer = data; cdc.TxLength = length; return USBD_OK; }
extern "C" uint8_t USBD_CDC_ReceivePacket(USBD_HandleTypeDef *) { armed_rx = cdc.RxBuffer; ++rx_rearms; return USBD_OK; }
extern "C" uint8_t USBD_CDC_TransmitPacket(USBD_HandleTypeDef *) { cdc.TxState = 1; ++tx_starts; return USBD_OK; }
static void Receive(uint8_t *data, uint16_t length)
{
    CHECK(length == 4);
    CHECK(armed_rx != data); // rearm must not overwrite the buffer being delivered
    memcpy(delivered, data, length);
    ++received;
}
static void Deliver(uint8_t value)
{
    memset(cdc.RxBuffer, value, 4);
    uint32_t length = 4;
    CHECK(USBD_Interface_fops_HS.Receive(cdc.RxBuffer, &length) == USBD_OK);
}
int main(int argc, char **argv)
{
    CHECK(argc == 2);
    hUsbDeviceHS.pClassData = &cdc;
    CHECK(USBD_Interface_fops_HS.Init() == USBD_OK);
    USB_Init(Receive);
    if (!strcmp(argv[1], "receive"))
    {
        Deliver(0x31);
        CHECK(received == 1 && delivered[0] == 0x31);
        Deliver(0x32);
        CHECK(received == 2 && delivered[0] == 0x32);
    }
    else if (!strcmp(argv[1], "reconnect"))
    {
        Deliver(0x41);
        CHECK(USBD_Interface_fops_HS.DeInit() == USBD_OK);
        CHECK(USBD_Interface_fops_HS.Init() == USBD_OK);
        CHECK(cdc.RxBuffer == UserRxBufferHS);
        Deliver(0x42);
        CHECK(received == 2 && delivered[0] == 0x42);
    }
    else if (!strcmp(argv[1], "startup"))
    {
        init_finished = false;
        Deliver(0x51);
        CHECK(received == 0);
        init_finished = true;
        Deliver(0x52);
        CHECK(received == 1 && delivered[0] == 0x52);
    }
    else if (!strcmp(argv[1], "transmit"))
    {
        uint8_t source[USB_BUFFER_SIZE];
        memset(source, 0x61, sizeof(source));
        CHECK(USB_Transmit_Data(source, sizeof(source)) == USBD_OK);
        memset(source, 0x62, sizeof(source));
        CHECK(cdc.TxLength == sizeof(source) && cdc.TxBuffer[0] == 0x61 && cdc.TxBuffer[511] == 0x61);
        CHECK(primask == 0);
    }
    else if (!strcmp(argv[1], "busy"))
    {
        uint8_t source[] = {0x71, 2, 3, 4};
        CHECK(USB_Transmit_Data(source, sizeof(source)) == USBD_OK);
        source[0] = 0x72; // EricTool packs its next frame before attempting to send
        CHECK(USB_Transmit_Data(source, sizeof(source)) == USBD_BUSY);
        CHECK(tx_starts == 1 && cdc.TxBuffer[0] == 0x71);
        cdc.TxState = 0; // IN completion
        primask = 1;
        CHECK(USB_Transmit_Data(source, sizeof(source)) == USBD_OK);
        CHECK(tx_starts == 2 && cdc.TxBuffer[0] == 0x72 && primask == 1);
    }
    else if (!strcmp(argv[1], "invalid"))
    {
        uint8_t source[USB_BUFFER_SIZE + 1] = {};
        CHECK(USB_Transmit_Data(nullptr, 4) == USBD_FAIL);
        CHECK(USB_Transmit_Data(source, 0) == USBD_FAIL);
        CHECK(USB_Transmit_Data(source, sizeof(source)) == USBD_FAIL);
        hUsbDeviceHS.pClassData = nullptr;
        CHECK(USB_Transmit_Data(source, 4) == USBD_FAIL);
        CHECK(tx_starts == 0 && primask == 0);
    }
    else { CHECK(false); }
    printf("PASS USB %s\n", argv[1]);
}

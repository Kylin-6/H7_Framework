#pragma once
#include <stdint.h>
#define USB_BUFFER_SIZE 512
struct Struct_USB_Manage_Object { uint8_t *Rx_Buffer_Ready; };
extern Struct_USB_Manage_Object USB0_Manage_Object;
uint8_t USB_Transmit_Data(uint8_t *, uint16_t);

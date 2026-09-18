#include "bsp_uart.h"
#include "dvc_erictool.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr, "UART TX check failed at line %d: %s\n", __LINE__, #condition); \
    exit(1); } } while (0)

volatile bool init_finished = true;
Class_Timestamp SYS_Timestamp;
Struct_USB_Manage_Object USB0_Manage_Object = {};
uint8_t USB_Transmit_Data(uint8_t *, uint16_t) { return HAL_OK; }

static uint32_t interrupt_mask;
static void (*barrier_hook)();
static HAL_StatusTypeDef dma_result = HAL_OK;
static HAL_StatusTypeDef blocking_result = HAL_OK;
static const uint8_t *dma_data;
static uint16_t dma_length;
static unsigned dma_calls;
static const uint8_t *blocking_data;
static uint32_t blocking_timeout;
static DMA_HandleTypeDef dma = {HAL_DMA_STATE_READY};
static DMA_HandleTypeDef dma2 = {HAL_DMA_STATE_READY};
static UART_HandleTypeDef uart1 = {USART1, nullptr, &dma, HAL_UART_STATE_READY, 0};
static UART_HandleTypeDef uart2 = {USART2, nullptr, &dma2, HAL_UART_STATE_READY, 0};

uint32_t __get_PRIMASK() { return interrupt_mask; }
void __disable_irq() { interrupt_mask = 1; }
void __set_PRIMASK(uint32_t value) { interrupt_mask = value; }
void __DMB()
{
    if (barrier_hook != nullptr)
    {
        void (*hook)() = barrier_hook;
        barrier_hook = nullptr;
        hook();
    }
}

HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t)
{
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *) { return HAL_OK; }

HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *uart, const uint8_t *data,
                                      uint16_t length)
{
    ++dma_calls;
    dma_data = data;
    dma_length = length;
    CHECK(uart->gState == HAL_UART_STATE_READY);
    if (dma_result == HAL_OK)
    {
        uart->gState = HAL_UART_STATE_BUSY_TX;
        uart->hdmatx->State = HAL_DMA_STATE_BUSY;
    }
    return dma_result;
}

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *, const uint8_t *data,
                                  uint16_t, uint32_t timeout)
{
    blocking_data = data;
    blocking_timeout = timeout;
    return blocking_result;
}

static void Check_Reentry()
{
    uint8_t replacement[] = {99, 98, 97};
    CHECK(interrupt_mask == 0);
    CHECK(uart1.gState == HAL_UART_STATE_READY);
    CHECK(UART_Transmit_Data(&uart1, replacement, sizeof(replacement)) == HAL_BUSY);
    CHECK(interrupt_mask == 0);
}

class Test_EricTool_UART : public Class_EricTool_UART
{
public:
    void Bind(float *value)
    {
        Data[0] = value;
        Data_Number = 1;
    }
};

int main()
{
    // NOLOAD memory must be initialized even for a TX-only port.
    memset(&USART1_Manage_Object, 0xa5, sizeof(USART1_Manage_Object));
    UART_Init(&uart1, nullptr);
    UART_Init(&uart2, nullptr);
    CHECK(!USART1_Manage_Object.Tx_Submitting);
    CHECK((uintptr_t)USART1_Manage_Object.Tx_Buffer % 32 == 0);

    uint8_t source[] = {1, 2, 3, 4};
    const uint8_t expected[] = {1, 2, 3, 4};
    barrier_hook = Check_Reentry;
    CHECK(UART_Transmit_Data(&uart1, source, sizeof(source)) == HAL_OK);
    CHECK(dma_data == USART1_Manage_Object.Tx_Buffer);
    CHECK(dma_data != source && dma_length == sizeof(source));
    CHECK(!USART1_Manage_Object.Tx_Submitting && interrupt_mask == 0);
    memset(source, 9, sizeof(source));
    CHECK(memcmp(dma_data, expected, sizeof(expected)) == 0);

    unsigned submitted = dma_calls;
    CHECK(UART_Transmit_Data(&uart1, source, sizeof(source)) == HAL_BUSY);
    CHECK(dma_calls == submitted);
    CHECK(memcmp(USART1_Manage_Object.Tx_Buffer, expected, sizeof(expected)) == 0);
    CHECK(UART_Transmit_Data(&uart2, source, sizeof(source)) == HAL_OK);
    CHECK(dma_data == USART2_Manage_Object.Tx_Buffer);
    CHECK(memcmp(USART1_Manage_Object.Tx_Buffer, expected, sizeof(expected)) == 0);

    // Simulate HAL completion/abort and failure to start, then retry.
    uart1.gState = HAL_UART_STATE_READY;
    CHECK(UART_Transmit_Data(&uart1, source, sizeof(source)) == HAL_BUSY);
    CHECK(memcmp(USART1_Manage_Object.Tx_Buffer, expected, sizeof(expected)) == 0);
    dma.State = HAL_DMA_STATE_READY;
    const HAL_StatusTypeDef failures[] = {HAL_ERROR, HAL_BUSY, HAL_TIMEOUT};
    for (HAL_StatusTypeDef result : failures)
    {
        dma_result = result;
        CHECK(UART_Transmit_Data(&uart1, source, sizeof(source)) == result);
        CHECK(!USART1_Manage_Object.Tx_Submitting);
        CHECK(uart1.gState == HAL_UART_STATE_READY);
    }
    dma_result = HAL_OK;
    uint8_t maximum[UART_BUFFER_SIZE];
    memset(maximum, 0x5a, sizeof(maximum));
    interrupt_mask = 1;
    CHECK(UART_Transmit_Data(&uart1, maximum, sizeof(maximum)) == HAL_OK);
    CHECK(interrupt_mask == 1);
    CHECK(memcmp(USART1_Manage_Object.Tx_Buffer, maximum, sizeof(maximum)) == 0);
    CHECK(UART_Transmit_Data(&uart1, source, sizeof(source)) == HAL_BUSY);
    CHECK(interrupt_mask == 1);
    interrupt_mask = 0;

    uart1.gState = HAL_UART_STATE_READY;
    dma.State = HAL_DMA_STATE_READY;
    submitted = dma_calls;
    UART_HandleTypeDef unsupported = {99, nullptr, &dma, HAL_UART_STATE_READY, 0};
    UART_HandleTypeDef uninitialized = {USART3, nullptr, &dma, HAL_UART_STATE_READY, 0};
    CHECK(UART_Transmit_Data(nullptr, source, sizeof(source)) == HAL_ERROR);
    CHECK(UART_Transmit_Data(&uart1, nullptr, sizeof(source)) == HAL_ERROR);
    CHECK(UART_Transmit_Data(&uart1, source, 0) == HAL_ERROR);
    CHECK(UART_Transmit_Data(&uart1, maximum, UART_BUFFER_SIZE + 1) == HAL_ERROR);
    CHECK(UART_Transmit_Data(&unsupported, source, sizeof(source)) == HAL_ERROR);
    CHECK(UART_Transmit_Data(&uninitialized, source, sizeof(source)) == HAL_ERROR);
    CHECK(dma_calls == submitted && interrupt_mask == 0);

    UART_HandleTypeDef blocking = {UART5, nullptr, nullptr, HAL_UART_STATE_READY, 0};
    blocking_result = HAL_TIMEOUT;
    CHECK(UART_Transmit_Data(&blocking, source, sizeof(source)) == HAL_TIMEOUT);
    CHECK(blocking_data == source && blocking_timeout == 10);
    CHECK(dma_calls == submitted);

    // Exercise the production EricTool packer without its 32-bit varargs API.
    Test_EricTool_UART telemetry;
    CHECK(telemetry.TIM_1ms_Write_PeriodElapsedCallback() == HAL_ERROR);
    telemetry.Init(&uart1);
    float value = 1.25f;
    telemetry.Bind(&value);
    CHECK(telemetry.TIM_1ms_Write_PeriodElapsedCallback() == HAL_OK);
    uint8_t frame[8];
    memcpy(frame, dma_data, sizeof(frame));
    CHECK(dma_length == sizeof(frame));
    CHECK(memcmp(frame, &value, sizeof(value)) == 0);
    uint32_t tail = 0x7f800000;
    CHECK(memcmp(frame + sizeof(value), &tail, sizeof(tail)) == 0);
    value = -2.5f;
    CHECK(telemetry.TIM_1ms_Write_PeriodElapsedCallback() == HAL_BUSY);
    CHECK(memcmp(USART1_Manage_Object.Tx_Buffer, frame, sizeof(frame)) == 0);
    uart1.gState = HAL_UART_STATE_READY;
    dma.State = HAL_DMA_STATE_READY;
    dma_result = HAL_ERROR;
    CHECK(telemetry.TIM_1ms_Write_PeriodElapsedCallback() == HAL_ERROR);
    dma_result = HAL_OK;
    CHECK(telemetry.TIM_1ms_Write_PeriodElapsedCallback() == HAL_OK);
    CHECK(memcmp(dma_data, &value, sizeof(value)) == 0);

    puts("UART DMA ownership, reentry, failure recovery and EricTool integration passed");
    return 0;
}

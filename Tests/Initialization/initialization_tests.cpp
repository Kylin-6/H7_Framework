#include "bsp_bmi088_accel.h"
#include "bsp_bmi088_gyro.h"
#include "bsp_bmi088.h"
#include "bsp_w25q64jv.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned checks;
#define CHECK(condition) do { ++checks; if (!(condition)) { \
    fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition); exit(1); } } while (0)

uint64_t test_time_us;
uint32_t test_pwm;
HAL_StatusTypeDef test_pwm_status = HAL_OK;
GPIO_TypeDef test_gpio;
TIM_HandleTypeDef htim3;
Class_Timestamp SYS_Timestamp;
Class_Power BSP_Power;
extern "C" { osThreadId_t BMI088TaskHandle = nullptr; }
static SPI_HandleTypeDef spi;
static OSPI_HandleTypeDef ospi;
Struct_SPI_Manage_Object SPI2_Manage_Object{&spi, {}, {}, 0};
Struct_OSPI_Manage_Object OSPI2_Manage_Object{&ospi, {}, {}, 0};

static Class_BMI088_Accel *accel;
static Class_BMI088_Gyro *gyro;
static Class_BMI088 *parent;
static int missing_sensor;
static unsigned spi_commands;
static bool is_accel, reset_seen, missing_after_reset, no_response;
static unsigned id_reads[2], writes[256], reads[256], ready_on_attempt;
static int rejected_register;
static uint8_t registers[256];
static uint8_t pending_address;
static uint16_t pending_length;
static bool spi_pending, flash_pending;
static unsigned flash_reads, ospi_commands, autopolls;
static HAL_StatusTypeDef mapped_result;
static HAL_StatusTypeDef ospi_result = HAL_OK, poll_result = HAL_OK;
static bool flash_auto_complete;

uint8_t SPI_Transmit_Data(SPI_HandleTypeDef *, GPIO_TypeDef *, uint16_t pin,
                          GPIO_PinState, const uint8_t *data, uint16_t length)
{
    ++spi_commands;
    is_accel = pin == BMI088_ACCEL__SPI_CS_Pin;
    CHECK(length == 2);
    ++writes[data[0]];
    if (data[0] == (is_accel ? 0x7e : 0x14) && data[1] == 0xb6)
    {
        reset_seen = true;
        memset(registers, 0, sizeof(registers));
        if (is_accel) registers[0x7c] = 3; // Reset leaves accel suspended.
    }
    else if (data[0] != rejected_register)
        registers[data[0]] = data[1];
    return HAL_OK;
}

uint8_t SPI_Transmit_Receive_Data(SPI_HandleTypeDef *, GPIO_TypeDef *port, uint16_t pin,
                                  GPIO_PinState, const uint8_t *data, uint16_t tx_length,
                                  uint16_t rx_length)
{
    ++spi_commands;
    is_accel = pin == BMI088_ACCEL__SPI_CS_Pin;
    SPI2_Manage_Object.Activate_GPIOx = port;
    SPI2_Manage_Object.Activate_GPIO_Pin = pin;
    CHECK(!spi_pending);
    memcpy(SPI2_Manage_Object.Tx_Buffer, data, tx_length);
    SPI2_Manage_Object.Rx_Buffer_Length = rx_length;
    pending_address = data[0] & 0x7f;
    pending_length = rx_length;
    ++reads[pending_address];
    spi_pending = true;
    return HAL_OK;
}

HAL_StatusTypeDef OSPI_Command_Receive_Data(OSPI_HandleTypeDef *, OSPI_RegularCmdTypeDef *command)
{
    ++ospi_commands;
    if (ospi_result != HAL_OK) return ospi_result;
    if (command->Instruction == 0x9f)
    {
        CHECK(command->NbData == 3);
        ++flash_reads;
        flash_pending = true;
    }
    else if (flash_auto_complete)
    {
        CHECK(command->DataMode == HAL_OSPI_DATA_1_LINE && command->NbData == 1);
        OSPI2_Manage_Object.Rx_Buffer[0] = command->Instruction == 0x35 ? 2 : 0;
        BSP_W25Q64JV.OSPI_RxCallback();
    }
    return HAL_OK;
}
HAL_StatusTypeDef OSPI_Command_Transmit_Data(OSPI_HandleTypeDef *, OSPI_RegularCmdTypeDef *) { ++ospi_commands; return ospi_result; }
HAL_StatusTypeDef OSPI_Command(OSPI_HandleTypeDef *, OSPI_RegularCmdTypeDef *) { ++ospi_commands; return ospi_result; }
HAL_StatusTypeDef OSPI_Auto_Polling(OSPI_HandleTypeDef *, OSPI_AutoPollingTypeDef *)
{
    ++autopolls;
    if (poll_result == HAL_OK && flash_auto_complete) BSP_W25Q64JV.OSPI_StatusMatchCallback();
    return poll_result;
}
HAL_StatusTypeDef HAL_OSPI_MemoryMapped(OSPI_HandleTypeDef *, OSPI_MemoryMappedTypeDef *)
{ return mapped_result; }

namespace Namespace_SYS_Timestamp {
void Delay_Millisecond(uint32_t milliseconds)
{
    test_time_us += milliseconds * 1000;
    CHECK(test_time_us < 30000000); // A missing device must return, not loop forever.
    if (spi_pending)
    {
        spi_pending = false;
        uint8_t value = registers[pending_address];
        if (pending_address == rejected_register) value = 0xff;
        if (pending_address == 0)
        {
            const unsigned attempt = ++id_reads[reset_seen ? 1 : 0];
            value = attempt >= ready_on_attempt && !(reset_seen && missing_after_reset)
                        ? (is_accel ? 0x1e : 0x0f) : 0;
            if ((missing_sensor & (is_accel ? 1 : 2)) != 0) value = 0;
        }
        if (!no_response)
        {
            memset(SPI2_Manage_Object.Rx_Buffer, 0, SPI_BUFFER_SIZE);
            SPI2_Manage_Object.Rx_Buffer[is_accel ? 2 : 1] = value;
            CHECK(pending_length <= 6);
            // Deliver through the actual driver callback only when the wait runs.
            if (parent) parent->SPI_RxCpltCallback();
            else if (is_accel) accel->SPI_RxCpltCallback();
            else gyro->SPI_RxCallback(0);
        }
    }
    if (flash_pending)
    {
        flash_pending = false;
        if (!no_response && flash_reads >= ready_on_attempt)
        {
            const uint8_t id[] = {0xef, 0x40, 0x17};
            memcpy(OSPI2_Manage_Object.Rx_Buffer, id, sizeof(id));
            BSP_W25Q64JV.OSPI_RxCallback();
        }
    }
}
}

static void Reset()
{
    test_time_us = 0;
    test_pwm = 123;
    test_pwm_status = HAL_OK;
    reset_seen = false;
    missing_after_reset = false;
    no_response = false;
    ready_on_attempt = 1;
    rejected_register = -1;
    missing_sensor = 0;
    parent = nullptr;
    spi_commands = 0;
    spi_pending = flash_pending = false;
    memset(id_reads, 0, sizeof(id_reads));
    memset(writes, 0, sizeof(writes));
    memset(reads, 0, sizeof(reads));
    memset(registers, 0, sizeof(registers));
    memset(OSPI2_Manage_Object.Rx_Buffer, 0, OSPI_BUFFER_SIZE);
    flash_reads = ospi_commands = autopolls = 0;
    mapped_result = HAL_OK;
}

static void TestSensor(bool accelerometer)
{
    // Normal; no ID; ID lost after reset; rejected nonzero/zero config;
    // ID on final allowed attempt; ID one attempt too late; no callbacks.
    for (int scenario = 0; scenario < 8; ++scenario)
    {
        Reset();
        is_accel = accelerometer;
        Class_BMI088_Accel accel_device;
        Class_BMI088_Gyro gyro_device;
        accel = &accel_device;
        gyro = &gyro_device;
        if (scenario == 1) ready_on_attempt = 100;
        if (scenario == 2) missing_after_reset = true;
        if (scenario == 3) rejected_register = is_accel ? 0x40 : 0x10;
        if (scenario == 4)
        {
            rejected_register = is_accel ? 0x7c : 0x0f;
            registers[rejected_register] = 0xff;
        }
        if (scenario == 5) ready_on_attempt = 5;
        if (scenario == 6) ready_on_attempt = 6;
        if (scenario == 7) no_response = true;
        const bool success = is_accel ? accel->Init(true) : gyro->Init();
        CHECK(success == (scenario == 0 || scenario == 5));
        CHECK(id_reads[0] <= 5 && id_reads[1] <= 5);
        if (scenario == 0 || scenario == 5)
        {
            CHECK(reset_seen);
            CHECK(writes[is_accel ? 0x7e : 0x14] == 1);
            CHECK(writes[is_accel ? 0x7c : 0x0f] == 1);
            CHECK(reads[is_accel ? 0x7c : 0x0f] == 1);
            if (is_accel) CHECK(registers[0x7c] == 0 && writes[0x7d] == 1);
        }
        if (scenario == 3 || scenario == 4) CHECK(writes[rejected_register] == 5);
        if (is_accel) CHECK(test_pwm == 0);
        printf("PASS %s scenario %d (ID reads %u/%u)\n",
               is_accel ? "accel" : "gyro", scenario, id_reads[0], id_reads[1]);
    }
}

static void TestPWMFailure()
{
    Reset();
    Class_BMI088_Accel device;
    test_pwm_status = HAL_ERROR;
    CHECK(!device.Init(true));
    CHECK(spi_commands == 0);
    printf("PASS PWM failure stops accel initialization\n");
}

static void CheckFlashUnavailable()
{
    const unsigned before = ospi_commands;
    uint8_t byte = 0;
    CHECK(!BSP_W25Q64JV.Is_Initialized());
    CHECK(!BSP_W25Q64JV.Is_Ready());
    CHECK(!BSP_W25Q64JV.Get_Buffer(0, 1));
    CHECK(!BSP_W25Q64JV.Set_Write_Enable());
    CHECK(!BSP_W25Q64JV.Set_Sector_Erased(0));
    CHECK(!BSP_W25Q64JV.Set_Buffer(&byte, 0, 1));
    CHECK(!BSP_W25Q64JV.Read_Data(&byte, 0, 1));
    CHECK(!BSP_W25Q64JV.Write_Data(&byte, 0, 1));
    BSP_W25Q64JV.Set_Bolck_Erased_32K(0);
    BSP_W25Q64JV.Set_Bolck_Erased_64K(0);
    BSP_W25Q64JV.Set_Chip_Erased();
    BSP_W25Q64JV.Enable_Quad_Mode();
    BSP_W25Q64JV.OSPI_RxCallback();
    BSP_W25Q64JV.OSPI_TxCallback();
    BSP_W25Q64JV.OSPI_StatusMatchCallback();
    BSP_W25Q64JV.TIM_1ms_AutoPollingTimeout_PeriodElapsedCallback();
    CHECK(ospi_commands == before && autopolls == 0);
}

static void TestBMI088()
{
    for (int failed = 0; failed < 4; failed++)
    {
        Reset();
        Class_BMI088 device;
        parent = &device;
        missing_sensor = failed;
        CHECK(!device.Is_Initialized());
        CHECK(device.Init() == (failed == 0));
        CHECK(device.Is_Initialized() == (failed == 0));
        if (failed)
        {
            const unsigned before = spi_commands;
            device.EXTI_Flag_Callback(BMI088_ACCEL__INTERRUPT_Pin);
            device.EXTI_Flag_Callback(BMI088_GYRO__INTERRUPT_Pin);
            device.TIM_1ms_Service_PeriodElapsedCallback();
            device.TIM_128ms_Calculate_PeriodElapsedCallback();
            device.BMI088_Service_Transfer(true);
            CHECK(spi_commands == before && test_pwm == 0);
        }
        printf("PASS BMI088 failed sensor mask %d\n", failed);
    }
}

static void TestFlash()
{
    CheckFlashUnavailable();
    for (int scenario = 0; scenario < 7; ++scenario)
    {
        Reset();
        if (scenario == 1) ready_on_attempt = 100;
        if (scenario == 2) ready_on_attempt = 5;
        if (scenario == 3) ready_on_attempt = 6;
        if (scenario == 4) mapped_result = HAL_ERROR;
        if (scenario == 5)
        {
            const uint8_t stale[] = {0xef, 0x40, 0x17};
            memcpy(OSPI2_Manage_Object.Rx_Buffer, stale, 3);
            no_response = true;
        }
        // A JEDEC response is exactly three bytes; the next byte is unrelated.
        OSPI2_Manage_Object.Rx_Buffer[3] = 0xaa;
        const bool success = BSP_W25Q64JV.Init(scenario == 4 || scenario == 6
            ? W25Q64JV_Mode_MemoryMapped : W25Q64JV_Mode_Normal);
        CHECK(success == (scenario == 0 || scenario == 2 || scenario == 6));
        CHECK(flash_reads >= 1 && flash_reads <= 5);
        CHECK(autopolls == 0); // ID callback must not overwrite the ID with status data.
        if (success) CHECK(BSP_W25Q64JV.Is_Initialized() && BSP_W25Q64JV.Is_Ready());
        else CheckFlashUnavailable();
        CHECK(OSPI2_Manage_Object.Rx_Buffer[3] == 0xaa);
        printf("PASS flash scenario %d (ID reads %u)\n", scenario, flash_reads);
    }
}

static void TestFlashSubmission()
{
    Reset();
    CHECK(BSP_W25Q64JV.Init());
    const HAL_StatusTypeDef failures[] = {HAL_ERROR, HAL_BUSY, HAL_TIMEOUT};
    for (unsigned index = 0; index < sizeof(failures) / sizeof(failures[0]); ++index)
    {
        HAL_StatusTypeDef failure = failures[index];
        unsigned before = BSP_W25Q64JV.Get_Auto_Polling_Error_Count();
        ospi_result = failure;
        CHECK(!BSP_W25Q64JV.Get_Buffer(0, 1));
        CHECK(BSP_W25Q64JV.Is_Ready());
        CHECK(BSP_W25Q64JV.Get_Auto_Polling_Error_Count() == before + 1);
        const unsigned polls_before = autopolls;
        CHECK(!BSP_W25Q64JV.Set_Write_Enable());
        CHECK(autopolls == polls_before && BSP_W25Q64JV.Is_Ready());
        ospi_result = HAL_OK;
        poll_result = failure;
        CHECK(!BSP_W25Q64JV.Set_Write_Enable());
        CHECK(BSP_W25Q64JV.Is_Ready());
        poll_result = HAL_OK;
        CHECK(BSP_W25Q64JV.Set_Write_Enable());
        BSP_W25Q64JV.OSPI_StatusMatchCallback();
        ospi_result = failure;
        uint8_t byte = 1;
        CHECK(!BSP_W25Q64JV.Set_Buffer(&byte, 0, 1));
        CHECK(BSP_W25Q64JV.Is_Ready());
        ospi_result = HAL_OK;
        CHECK(BSP_W25Q64JV.Set_Write_Enable());
        BSP_W25Q64JV.OSPI_StatusMatchCallback();
        ospi_result = failure;
        CHECK(!BSP_W25Q64JV.Set_Sector_Erased(0));
        CHECK(BSP_W25Q64JV.Is_Ready());
        const unsigned commands_before = ospi_commands;
        BSP_W25Q64JV.Enable_Quad_Mode();
        CHECK(ospi_commands == commands_before + 1 && BSP_W25Q64JV.Is_Ready());
        ospi_result = HAL_OK;
    }
}

static void TestFlashQuad()
{
    Reset();
    CHECK(BSP_W25Q64JV.Init());
    flash_auto_complete = true;
    const unsigned errors_before = BSP_W25Q64JV.Get_Auto_Polling_Error_Count();
    BSP_W25Q64JV.Enable_Quad_Mode();
    CHECK(BSP_W25Q64JV.Is_Ready());
    CHECK(BSP_W25Q64JV.Get_Auto_Polling_Error_Count() == errors_before);
    CHECK(autopolls == 1 && ospi_commands == 10);
}

int main(int argc, char **argv)
{
    if (argc != 2) return 2;
    if (strcmp(argv[1], "accel") == 0) { TestSensor(true); TestPWMFailure(); }
    else if (strcmp(argv[1], "gyro") == 0) TestSensor(false);
    else if (strcmp(argv[1], "bmi088") == 0) TestBMI088();
    else if (strcmp(argv[1], "flash") == 0) TestFlash();
    else if (strcmp(argv[1], "flash_submission") == 0) TestFlashSubmission();
    else if (strcmp(argv[1], "flash_quad") == 0) TestFlashQuad();
    else return 2;
    printf("PASS %s: %u checks\n", argv[1], checks);
    return 0;
}

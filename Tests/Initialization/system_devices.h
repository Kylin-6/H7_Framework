#pragma once
extern bool test_imu_ready, test_flash_ready, test_adc_ready;
enum { HAL_OK, HAL_ERROR };
extern int test_tim4_status, test_tim5_status;
extern unsigned test_fifo_starts, test_other_inits;
struct TestGyro { void Start_FIFO_Acquisition() { ++test_fifo_starts; } };
struct TestIMU {
    TestGyro BMI088_Gyro;
    bool Init() { return test_imu_ready; }
    bool Is_Initialized() const { return test_imu_ready; }
};
struct TestFlash { bool Init() { return test_flash_ready; } };
struct TestDevice { void Init(bool = false, bool = false, bool = false) { ++test_other_inits; } };
struct TestClock { void Init(void *) {} };
extern TestIMU BSP_BMI088;
extern TestFlash BSP_W25Q64JV;
extern TestDevice BSP_WS2812, BSP_Buzzer, BSP_Key, BSP_Power, EricTool_USB;
extern TestClock SYS_Timestamp;
extern int htim4, htim5, hspi2, hspi6, hospi2, hadc1;
extern int huart1, huart2, huart3, huart4, huart5, huart6, huart7, huart8, huart9, huart10;
enum { EXTI15_10_IRQn };
inline void SEGGER_SYSVIEW_Conf() {}
inline void HAL_NVIC_SetPriority(int, int, int) {}
inline int HAL_TIM_Base_Start_IT(void *timer)
{ return timer == &htim4 ? test_tim4_status : test_tim5_status; }
inline void UART_Init(void *, void *) {}
inline void SPI_Init(void *, void (*)()) {}
inline void OSPI_Init(void *, void (*)(), void (*)(), void (*)()) {}
inline bool ADC_Init(void *, int) { return test_adc_ready; }
inline void System_IMU_Configure() {}
inline void SPI2_Callback() {}
inline void OSPI2_Polling_Callback() {}
inline void OSPI2_Rx_Callback() {}
inline void OSPI2_Tx_Callback() {}

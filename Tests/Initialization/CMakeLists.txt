cmake_minimum_required(VERSION 3.22)
project(H7_BSP_Initialization_Tests LANGUAGES CXX)
get_filename_component(BSP_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
set(ONBOARD "${BSP_ROOT}/User_File/Device/Onboard")
set(ALGORITHM "${BSP_ROOT}/User_File/Middleware/Algorithm")
add_executable(initialization_tests
    initialization_tests.cpp
    ${ONBOARD}/BMI088/bsp_bmi088.cpp
    ${ONBOARD}/BMI088/Accel/bsp_bmi088_accel.cpp
    ${ONBOARD}/BMI088/Gyro/bsp_bmi088_gyro.cpp
    ${ONBOARD}/W25Q64JV/bsp_w25q64jv.cpp
    ${ALGORITHM}/Basic/alg_basic.cpp
    ${ALGORITHM}/Matrix/alg_matrix.cpp
    ${ALGORITHM}/PID/alg_pid.cpp
    ${ALGORITHM}/Filter/IIR/alg_filter_iir.cpp
    ${ALGORITHM}/Filter/VQF/alg_filter_vqf.cpp
    ${ALGORITHM}/Quaternion/alg_quaternion.cpp
)
target_compile_features(initialization_tests PRIVATE cxx_std_17)
target_include_directories(initialization_tests PRIVATE
    stubs ../Boundary/stubs
    ${ONBOARD}/BMI088 ${ONBOARD}/BMI088/Accel ${ONBOARD}/BMI088/Gyro ${ONBOARD}/W25Q64JV
    ${ALGORITHM}/Basic ${ALGORITHM}/Matrix ${ALGORITHM}/PID ${ALGORITHM}/Filter/IIR
    ${ALGORITHM}/Filter/VQF ${ALGORITHM}/Quaternion
)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(initialization_tests PRIVATE -Wall -Wextra -Werror=return-type)
endif()
enable_testing()
foreach(group accel gyro bmi088 flash flash_submission flash_quad)
    add_test(NAME initialization_${group} COMMAND initialization_tests ${group})
    set_tests_properties(initialization_${group} PROPERTIES TIMEOUT 10)
endforeach()

# Compile the real startup sequence with controllable device Init results.
set(SYSTEM_STUBS "${CMAKE_CURRENT_BINARY_DIR}/system_stubs")
file(MAKE_DIRECTORY "${SYSTEM_STUBS}")
foreach(header Com SEGGER_SYSVIEW bsp_adc bsp_bmi088 bsp_buzzer bsp_can bsp_key
        bsp_ospi bsp_power bsp_spi bsp_uart bsp_w25q64jv bsp_ws2812 callback
        dvc_erictool sys_debug sys_imu sys_timestamp usart)
    file(WRITE "${SYSTEM_STUBS}/${header}.h" "#pragma once\n#include \"system_devices.h\"\n")
endforeach()
add_executable(system_init_tests system_init_tests.cpp
    ${BSP_ROOT}/User_File/System/Init/Init.cpp)
target_compile_features(system_init_tests PRIVATE cxx_std_17)
target_include_directories(system_init_tests PRIVATE ${SYSTEM_STUBS}
    ${CMAKE_CURRENT_LIST_DIR} ${BSP_ROOT}/User_File/System/Init)
add_test(NAME initialization_system COMMAND system_init_tests)
set_tests_properties(initialization_system PROPERTIES TIMEOUT 10)

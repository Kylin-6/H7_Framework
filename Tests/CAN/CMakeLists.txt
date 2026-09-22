cmake_minimum_required(VERSION 3.22)
project(H7_BSP_CAN_Tests LANGUAGES C)

get_filename_component(BSP_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
add_executable(can_tests
    can_tests.c
    ${BSP_ROOT}/User_File/Middleware/BSP/CAN/bsp_can.c
)
target_include_directories(can_tests PRIVATE
    stubs
    ${BSP_ROOT}/User_File/Middleware/BSP/CAN
)
enable_testing()
add_test(NAME can_tx_stats COMMAND can_tests)

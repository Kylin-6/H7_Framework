cmake_minimum_required(VERSION 3.22)
project(H7_BSP_Boundary_Tests LANGUAGES CXX)

# Standalone host tests: do not load the firmware's ARM toolchain.
get_filename_component(BSP_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
set(ALGORITHM "${BSP_ROOT}/User_File/Middleware/Algorithm")
set(PERIPHERAL "${BSP_ROOT}/User_File/Device/Peripheral")

add_executable(boundary_tests
    boundary_tests.cpp
    ${ALGORITHM}/Basic/alg_basic.cpp
    ${ALGORITHM}/Matrix/alg_matrix.cpp
    ${ALGORITHM}/PID/alg_pid.cpp
    ${PERIPHERAL}/EricTool/dvc_erictool.cpp
    ${PERIPHERAL}/Motor/DMmotor/dmmotor.cpp
    ${PERIPHERAL}/Motor/QDrive/QD4310.cpp
)
target_compile_features(boundary_tests PRIVATE cxx_std_17)
target_include_directories(boundary_tests PRIVATE
    stubs
    ${ALGORITHM}/Basic
    ${ALGORITHM}/Matrix
    ${ALGORITHM}/PID
    ${ALGORITHM}/Filter/Kalman
    ${PERIPHERAL}/EricTool
    ${PERIPHERAL}/Motor/DMmotor
    ${PERIPHERAL}/Motor/QDrive
)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(boundary_tests PRIVATE -Wall -Wextra -Werror=return-type)
endif()

enable_testing()
foreach(group pid kalman commands parser)
    add_test(NAME boundary_${group} COMMAND boundary_tests ${group})
endforeach()

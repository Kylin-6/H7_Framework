cmake_minimum_required(VERSION 3.22)
project(H7_BSP_Gimbal_Tests LANGUAGES CXX)
get_filename_component(ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
set(USER "${ROOT}/User_File")
set(ALG "${USER}/Middleware/Algorithm")
add_executable(gimbal_tests gimbal_tests.cpp
    ${USER}/Application/Gimbal/Gimbal.cpp
    ${USER}/Device/Peripheral/Motor/DMmotor/dmmotor.cpp
    ${USER}/System/Daemon/daemon.cpp
    ${USER}/System/MessageCenter/message_center.cpp
    ${ALG}/Basic/alg_basic.cpp ${ALG}/Matrix/alg_matrix.cpp
    ${ALG}/PID/alg_pid.cpp ${ALG}/Filter/IIR/alg_filter_iir.cpp)
target_compile_features(gimbal_tests PRIVATE cxx_std_17)
target_compile_definitions(gimbal_tests PRIVATE GIMBAL=1)
target_include_directories(gimbal_tests PRIVATE
    ${USER}/System/Daemon stubs ../Boundary/stubs
    ${USER}/Application/Gimbal ${USER}/Device/Peripheral/Motor/DMmotor
    ${USER}/System/MessageCenter ${ALG}/Basic ${ALG}/Matrix ${ALG}/PID ${ALG}/Filter/IIR)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(gimbal_tests PRIVATE -Wall -Wextra -Werror=return-type)
endif()
enable_testing()
foreach(group config init_failure partial_init stable control pitch direction modes recovery timeout failures invalid driver defaults)
    add_test(NAME gimbal_${group} COMMAND gimbal_tests ${group})
    set_tests_properties(gimbal_${group} PROPERTIES TIMEOUT 10)
endforeach()

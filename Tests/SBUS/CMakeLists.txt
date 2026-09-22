cmake_minimum_required(VERSION 3.16)
project(sbus_tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(sbus_tests
    sbus_tests.cpp
    ../../User_File/Device/Peripheral/Remote/sbus.cpp
)

target_include_directories(sbus_tests PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/../../User_File/Device/Peripheral/Remote
)

target_compile_options(sbus_tests PRIVATE -Wall -Wextra -Werror)

enable_testing()
add_test(NAME sbus_tests COMMAND sbus_tests)

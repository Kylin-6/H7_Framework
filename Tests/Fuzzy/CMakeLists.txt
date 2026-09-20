cmake_minimum_required(VERSION 3.22)
project(H7_BSP_Fuzzy_Tests LANGUAGES CXX)

get_filename_component(BSP_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
set(ALGORITHM "${BSP_ROOT}/User_File/Middleware/Algorithm")

add_executable(fuzzy_tests
    fuzzy_tests.cpp
    ${ALGORITHM}/Fuzzy/alg_fuzzy.cpp
    ${ALGORITHM}/Basic/alg_basic.cpp
)
target_compile_features(fuzzy_tests PRIVATE cxx_std_11)
target_include_directories(fuzzy_tests PRIVATE stubs ${ALGORITHM}/Basic ${ALGORITHM}/Fuzzy)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(fuzzy_tests PRIVATE -Wall -Wextra -Werror)
endif()

enable_testing()
add_test(NAME fuzzy_sugeno COMMAND fuzzy_tests)

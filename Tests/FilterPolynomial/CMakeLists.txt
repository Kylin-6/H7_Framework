cmake_minimum_required(VERSION 3.22)
project(H7_BSP_FilterPolynomial_Tests LANGUAGES CXX)

get_filename_component(BSP_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
set(ALGORITHM "${BSP_ROOT}/User_File/Middleware/Algorithm")

add_executable(filter_polynomial_tests
    filter_polynomial_tests.cpp
    ${ALGORITHM}/Filter/Polynomial/alg_filter_polynomial.cpp
    ${ALGORITHM}/Basic/alg_basic.cpp
)
target_compile_features(filter_polynomial_tests PRIVATE cxx_std_11)
target_include_directories(filter_polynomial_tests PRIVATE
    ../Fuzzy/stubs ${ALGORITHM}/Basic ${ALGORITHM}/Filter/Polynomial)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(filter_polynomial_tests PRIVATE -Wall -Wextra -Werror)
endif()

enable_testing()
foreach(group reference stream polynomial lifecycle boundary)
    add_test(NAME filter_polynomial_${group} COMMAND filter_polynomial_tests ${group})
endforeach()

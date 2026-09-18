cmake_minimum_required(VERSION 3.22)
project(H7_BSP_Trajectory_Tests LANGUAGES CXX)

get_filename_component(BSP_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
set(ALGORITHM "${BSP_ROOT}/User_File/Middleware/Algorithm")

add_executable(trajectory_tests
    trajectory_tests.cpp
    ${ALGORITHM}/Trajectory/alg_trajectory.cpp
    ${ALGORITHM}/Basic/alg_basic.cpp
)
target_compile_features(trajectory_tests PRIVATE cxx_std_11)
target_include_directories(trajectory_tests PRIVATE
    ../Fuzzy/stubs ${ALGORITHM}/Basic ${ALGORITHM}/Trajectory)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(trajectory_tests PRIVATE -Wall -Wextra -Werror)
endif()

enable_testing()
add_test(NAME trajectory_contract COMMAND trajectory_tests contract)
add_test(NAME trajectory_random COMMAND trajectory_tests random)
add_test(NAME trajectory_scale COMMAND trajectory_tests scale)
add_test(NAME trajectory_boundary COMMAND trajectory_tests boundary)
add_test(NAME trajectory_retarget COMMAND trajectory_tests retarget)

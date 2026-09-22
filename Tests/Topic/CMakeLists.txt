cmake_minimum_required(VERSION 3.22)
project(H7_BSP_Topic_Tests LANGUAGES CXX)

get_filename_component(BSP_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
add_executable(topic_tests topic_tests.cpp)
target_compile_features(topic_tests PRIVATE cxx_std_17)
target_include_directories(topic_tests PRIVATE
    stubs
    ${BSP_ROOT}/User_File/System/MessageCenter
)
enable_testing()
add_test(NAME topic_freshness COMMAND topic_tests)

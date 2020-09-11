cmake_minimum_required(VERSION 3.8)
project(pidtest VERSION 0.1)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 11)

include(CTest)

enable_testing()

add_executable(pidtest test.cpp)
target_link_libraries(pidtest PUBLIC gtest_main pid)

add_test(NAME pidtest COMMAND pidtest)
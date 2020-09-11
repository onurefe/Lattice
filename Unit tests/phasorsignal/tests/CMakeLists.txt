cmake_minimum_required(VERSION 3.8)
project(phasorsignaltest VERSION 0.1)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 11)

include(CTest)

enable_testing()

add_executable(phasorsignaltest test.cpp)
target_link_libraries(phasorsignaltest PUBLIC gtest_main phasorsignal)

add_test(NAME phasorsignaltest COMMAND phasorsignaltest)
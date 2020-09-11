cmake_minimum_required(VERSION 3.8)
project(regressiontest VERSION 0.1)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 11)

include(CTest)

enable_testing()

add_executable(regressiontest test.cpp)
target_link_libraries(regressiontest PUBLIC gtest_main regression)

add_test(NAME regressiontest COMMAND regressiontest)

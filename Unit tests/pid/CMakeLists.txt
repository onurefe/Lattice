cmake_minimum_required(VERSION 3.8)
project(pid VERSION 0.1)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 11)

include(CTest)
enable_testing()

add_subdirectory(googletest)
add_library(pid pid/src/pid.c pid/inc/pid.h)
add_subdirectory(tests)

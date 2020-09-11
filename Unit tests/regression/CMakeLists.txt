cmake_minimum_required(VERSION 3.8)
project(regression VERSION 0.1)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 11)

include(CTest)
enable_testing()

add_subdirectory(googletest)
add_library(regression regression/src/polyreg.c regression/src/matrix.c regression/inc/polyreg.h regression/inc/matrix.h)
add_subdirectory(tests)

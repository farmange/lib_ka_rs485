cmake_minimum_required (VERSION 2.6)
project (lib_ka_rs485_examples)

# set(CMAKE_VERBOSE_MAKEFILE ON)

include_directories(${CMAKE_SOURCE_DIR}/include)
set(LIBKARS485_LIBRARIES "${CMAKE_SOURCE_DIR}/build/src/libka_rs485.so")

add_executable (ka_rs485_test test.cpp)

target_link_libraries (ka_rs485_test ${LIBKARS485_LIBRARIES})

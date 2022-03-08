cmake_minimum_required (VERSION 2.6)
project (lib_ka_rs485)

# The version number.
set (lib_ka_rs485_VERSION_MAJOR 0)
set (lib_ka_rs485_VERSION_MINOR 1)
set (lib_ka_rs485_VERSION_PATCH 0)
set (lib_ka_rs485_VERSION ${lib_ka_rs485_VERSION_MAJOR}.${lib_ka_rs485_VERSION_MINOR}.${lib_ka_rs485_VERSION_PATCH})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall -Werror -std=c++11")

add_subdirectory (src)

add_subdirectory (examples)

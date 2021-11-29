cmake_minimum_required (VERSION 2.6)

# include directory containing header files
include_directories (${lib_ka_rs485_SOURCE_DIR}/include)

# set as shared library
add_library (ka_rs485 SHARED ka_rs485.cpp logger.cpp)

if (DEBUG)
MESSAGE(STATUS "Debug mode. All message will be stub with empty (zero) values.")
add_definitions(-DDEBUG_STUB_MSG -shared)
endif()

set_target_properties(
  ka_rs485
  PROPERTIES
    SOVERSION ${lib_ka_rs485_VERSION_MAJOR}
    VERSION ${lib_ka_rs485_VERSION}
)

# link external libraries
target_link_libraries (ka_rs485 pthread dl)


# set the install targets
install (TARGETS ka_rs485 LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}")
file (GLOB ka_rs485_header_files ${lib_ka_rs485_SOURCE_DIR}/include/*.h)
install (FILES ${ka_rs485_header_files} DESTINATION include/ka_rs485)

# g++ -std=c++11 -L/usr/local/lib test2.cpp -o test2 -Wall -ldl -lpthread -lka_rs485
# LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH ./test2 -a 22 -v 1
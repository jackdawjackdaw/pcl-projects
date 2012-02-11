cmake_minimum_required (VERSION 2.6)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}")
project (Emulator)

MESSAGE("C Compiler: ${CMAKE_C_COMPILER}")

if(${CMAKE_BUILD_TYPE} MATCHES "debug")
 MESSAGE("C DEBUG FLAGS: ${CMAKE_C_FLAGS_DEBUG}")
else()
 MESSAGE("C FLAGS: ${CMAKE_C_FLAGS}")
endif(${CMAKE_BUILD_TYPE} MATCHES "debug")

add_subdirectory(src)


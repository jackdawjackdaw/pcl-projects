cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcd_write)

#set(CMAKE_C_COMPILER "clang")
#set(CMAKE_CXX_COMPILER "g++")

#set(CMAKE_VERBOSE_MAKEFILE TRUE)

MESSAGE("C Compiler: ${CMAKE_C_COMPILER}")
MESSAGE("C++ Compiler: ${CMAKE_CXX_COMPILER}")

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pcd_write pcd_write.cpp)
target_link_libraries (pcd_write ${PCL_LIBRARIES})

add_executable (pcl_cluster cluster_extract.cpp)
target_link_libraries (pcl_cluster ${PCL_LIBRARIES})

add_executable (pcl_hull convex_hull.cpp)
target_link_libraries (pcl_hull ${PCL_LIBRARIES})

add_executable (test_io test_io.cpp)
target_link_libraries (test_io ${PCL_LIBRARIES})

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcd_write)

#set(CMAKE_VERBOSE_MAKEFILE TRUE)

#set(CMAKE_CXX_COMPILER g++-mp-4.4)

MESSAGE("C Compiler: ${CMAKE_C_COMPILER}")
MESSAGE("C++ Compiler: ${CMAKE_CXX_COMPILER}")


set(CMAKE_CXX_FLAGS "-Wall -g -pg -fprofile-arcs")

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pcd_write pcd_write.cpp)
target_link_libraries (pcd_write ${PCL_LIBRARIES})

add_executable (pcl_cluster cluster_extract.cpp binfile.cpp)
target_link_libraries (pcl_cluster ${PCL_LIBRARIES})

add_executable (fit_planes fit_planes.cpp binfile.cpp)
target_link_libraries (fit_planes ${PCL_LIBRARIES})

add_executable (bin2pcd bin2pcd.cpp binfile.cpp)
target_link_libraries (bin2pcd ${PCL_LIBRARIES})

add_executable (pcd2bin pcd2bin.cpp binfile.cpp)
target_link_libraries (pcd2bin ${PCL_LIBRARIES})


add_executable (ransac ransac_cubes.cpp binfile.cpp)
target_link_libraries (ransac ${PCL_LIBRARIES})

add_executable (pcl_hull convex_hull.cpp)
target_link_libraries (pcl_hull ${PCL_LIBRARIES})

#add_executable (test_io test_io.cpp)
#target_link_libraries (test_io ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})

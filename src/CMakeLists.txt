cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcd_write)

## this is kind of irritating
#set(CMAKE_VERBOSE_MAKEFILE TRUE)

set(CMAKE_CXX_COMPILER g++)

MESSAGE("C Compiler: ${CMAKE_C_COMPILER}")
MESSAGE("C++ Compiler: ${CMAKE_CXX_COMPILER}")


## debug flags on 
set(CMAKE_CXX_FLAGS "-Wall -O0 -g")

message("cmake c++ flags: ${CMAKE_CXX_FLAGS}")

message("boost include: ${Boost_INCLUDE_DIRS}")
message("boost lib: ${Boost_LIBRARY_DIRS}")

find_package(PCL 1.2 REQUIRED)

set(BOOST_ROOT "~/local/boost_1_48_0")
set(Boost_INCLUDE_DIRS "~/local/boost_1_48_0/")
set(Boost_LIBRARY_DIRS "~/local/boost_1_48_0/stage/lib")

include_directories(${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
#link_directories(${Boost_LIBRARY_DIRS} ${PCL_LIBRARY_DIRS})
											 
message("boost include: ${Boost_INCLUDE_DIRS}")
message("boost lib: ${Boost_LIBRARY_DIRS}")
message("pcl libs: ${PCL_LIBRARY_DIRS}")
message("pcl include: ${PCL_INCLUDE_DIRS}")


add_definitions(${PCL_DEFINITIONS})

add_executable (pcd_write pcd_write.cpp)
target_link_libraries (pcd_write ${PCL_LIBRARIES})

add_executable (cluster cluster_extract.cpp binfile.cpp)
target_link_libraries (cluster ${PCL_LIBRARIES})

add_executable (fit_planes fit_planes.cpp binfile.cpp)
target_link_libraries (fit_planes ${PCL_LIBRARIES})

add_executable (bin2pcd bin2pcd.cpp binfile.cpp)
target_link_libraries (bin2pcd ${PCL_LIBRARIES})

add_executable (pcd2bin pcd2bin.cpp binfile.cpp)
target_link_libraries (pcd2bin ${PCL_LIBRARIES})

add_executable (icpcubes icp_cubes.cpp binfile.cpp)
target_link_libraries (icpcubes ${PCL_LIBRARIES})

add_executable (pcl_hull convex_hull.cpp)
target_link_libraries (pcl_hull ${PCL_LIBRARIES})

add_executable (full_cloud_register full_cloud_register.cpp binfile.cpp)
target_link_libraries (full_cloud_register ${PCL_LIBRARIES})							 

add_executable (test_io test_io.cpp)
target_link_libraries (test_io ${PCL_LIBRARIES})

add_executable (features features.cpp)
target_link_libraries (features  ${PCL_LIBRARIES})



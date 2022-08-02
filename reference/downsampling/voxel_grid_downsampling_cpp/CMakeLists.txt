cmake_minimum_required(VERSION 3.16.3 FATAL_ERROR)

project(voxel_grid)
 
set(CMAKE_CXX_STANDARD 11)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (voxel_grid_downsampling voxel_grid_downsampling.cpp)
target_link_libraries (voxel_grid_downsampling ${PCL_LIBRARIES})
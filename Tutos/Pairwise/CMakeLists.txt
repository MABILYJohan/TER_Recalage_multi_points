cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(pcl-meta)

find_package(PCL 1.1 REQUIRED)

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pair main.cpp)
target_link_libraries (pair ${PCL_LIBRARIES})

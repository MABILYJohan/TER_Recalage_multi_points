cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(pcl-kfpcs)

find_package(PCL 1.1 REQUIRED)

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (KFPCS main.cpp)
target_link_libraries (KFPCS ${PCL_LIBRARIES})

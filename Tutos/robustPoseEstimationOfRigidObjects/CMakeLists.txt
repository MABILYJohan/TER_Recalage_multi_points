cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(alignment_prerejective)

find_package(PCL 1.3 REQUIRED)
find_package(PCL 1.7 REQUIRED REQUIRED COMPONENTS io registration segmentation visualization)

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(manager main.cpp )
target_link_libraries(manager ${PCL_LIBRARIES})

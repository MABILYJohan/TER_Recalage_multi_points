cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(iterative_closest_point)

find_package(PCL 1.2 REQUIRED)
find_package(PCL 1.3 REQUIRED)

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(manager main.cpp )
target_link_libraries(manager ${PCL_LIBRARIES})

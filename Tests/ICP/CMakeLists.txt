cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(pcl-interactive_icp)

find_package(PCL 1.5 REQUIRED)

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (icp main.cpp)
target_link_libraries (icp ${PCL_LIBRARIES})

add_executable (interactive_icp interactiveICP.cpp)
target_link_libraries (interactive_icp ${PCL_LIBRARIES})

cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(pcl-interactive_icp)

find_package(PCL 1.7 REQUIRED)

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (test_registration test_registration.cpp)
target_link_libraries (test_registration ${PCL_LIBRARIES})

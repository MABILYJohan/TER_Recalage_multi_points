cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(solution)

find_package(PCL 1.3 REQUIRED)

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(vizu vizu.cpp )
target_link_libraries(vizu ${PCL_LIBRARIES})

add_executable(vizuDS vizuDownSample.cpp )
target_link_libraries(vizuDS ${PCL_LIBRARIES})

add_executable(vizuCC compare2Clouds.cpp )
target_link_libraries(vizuCC ${PCL_LIBRARIES})


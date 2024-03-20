cmake_minimum_required(VERSION 3.22)
project(automatic_reduction_of_mesh)

find_package(PCL 1.2 REQUIRED)

set(CMAKE_CXX_STANDARD 20)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(automatic_reduction_of_mesh main.cpp DisparityMap.cpp DisparityMap.h Triangulation.cpp Triangulation.h Visualisation.cpp Visualisation.h MeshReduction.cpp MeshReduction.h)
target_link_libraries (automatic_reduction_of_mesh ${PCL_LIBRARIES})

cmake_minimum_required(VERSION 3.5 FATAL_ERROR)


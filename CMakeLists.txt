cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(pcl_demo)
find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#
find_package(OpenCV REQUIRED)

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

add_executable (pcl_demo main.cpp surface_fitting.cpp pcl_expansion.cpp)
target_link_libraries (pcl_demo ${PCL_LIBRARIES})
target_link_libraries(pcl_demo ${OpenCV_LIBS})
target_link_libraries(pcl_demo ${CERES_LIBRARIES})
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(self_adaptive_outlier_removal)

# PCL
find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Opencv
find_package(OpenCV REQUIRED)

# Ceres
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

# gLib
include_directories(gLib .)
ADD_SUBDIRECTORY(gLib)

add_executable (self_adaptive_outlier_removal main.cpp euclidean_distance.cpp)
# PCL
target_link_libraries (self_adaptive_outlier_removal ${PCL_LIBRARIES})

# OpenCV
target_link_libraries(self_adaptive_outlier_removal ${OpenCV_LIBS})

# Ceres
target_link_libraries(self_adaptive_outlier_removal ${CERES_LIBRARIES})

#gLib
target_link_libraries(self_adaptive_outlier_removal GLIB)
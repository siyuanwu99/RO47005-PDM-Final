cmake_minimum_required(VERSION 3.0.2)
project(map_server)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11)
add_compile_options(-std=c++14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pcl_conversions
  pcl_ros
  sensor_msgs
  geometry_msgs
)

## System dependencies are found with CMake's conventions

find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES map_server
 CATKIN_DEPENDS pcl_conversions pcl_ros sensor_msgs roscpp
 DEPENDS system_lib PCL EIGEN3
)

###########
## Build ##
###########
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  SYSTEM
  include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${Eigen3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(map_server
  src/grid_map.cpp
)

target_link_libraries(map_server
${catkin_LIBRARIES}
)

add_executable(test_grid_map_node src/test_grid_map_node.cpp)
target_link_libraries(test_grid_map_node
  ${catkin_LIBRARIES}
  map_server
)

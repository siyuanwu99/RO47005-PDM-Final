cmake_minimum_required(VERSION 3.0.2)
project(planner)
add_compile_options(-std=c++14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
find_package(catkin REQUIRED COMPONENTS
  roscpp
  quadrotor_msgs
  map_server  
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES probabilistic_roadmap
    CATKIN_DEPENDS map_server quadrotor_msgs roscpp
    DEPENDS EIGEN3
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  include
  SYSTEM
)

add_library(probabilistic_roadmap
  src/prm_planner.cpp
)
add_executable(${PROJECT_NAME}_node src/simple_prm.cpp)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  probabilistic_roadmap
  map_server
)
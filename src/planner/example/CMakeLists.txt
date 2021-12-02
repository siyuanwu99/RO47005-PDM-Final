cmake_minimum_required(VERSION 3.0.2)
project(planner_example)
add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
find_package(catkin REQUIRED COMPONENTS
  roscpp
  quadrotor_msgs
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

catkin_package(
     INCLUDE_DIRS include
  #   CATKIN_DEPENDS 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  include
)

add_executable(${PROJECT_NAME}_node src/simple_prm.cpp src/prm_planner.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
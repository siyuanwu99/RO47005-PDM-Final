cmake_minimum_required(VERSION 2.8.3)
project(traj_opt)
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
)
catkin_package(

)

find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

add_library(${PROJECT_NAME}
  src/poly_opt.cpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
)

add_executable(test_traj_opt_node  src/test_traj_opt_node.cpp)

target_link_libraries(test_traj_opt_node
   ${catkin_LIBRARIES}
   ${PROJECT_NAME}
)
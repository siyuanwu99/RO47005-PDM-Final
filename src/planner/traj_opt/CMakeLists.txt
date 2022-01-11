cmake_minimum_required(VERSION 2.8.3)
project(traj_opt)
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  quadrotor_msgs
  map_server  
  decomp_ros_utils
  pcl_conversions
  pcl_ros
)
catkin_package(

)

find_package(Eigen3 REQUIRED)
find_package(osqp REQUIRED)
find_package(PCL REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
  ${oscp_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/poly_opt.cpp
  src/corridor_minisnap.cpp
)

add_executable(test_traj_opt_node  src/test_traj_opt_node.cpp)
add_executable(compare_traj  src/compare_traj.cpp)
add_executable(traj_opt_node  src/traj_opt_node.cpp)

target_link_libraries(test_traj_opt_node
   ${catkin_LIBRARIES}
   ${PROJECT_NAME}
   osqp::osqp
)
target_link_libraries(traj_opt_node
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${PROJECT_NAME}
   osqp::osqp
)
target_link_libraries(compare_traj
   ${catkin_LIBRARIES}
   ${PROJECT_NAME}
   osqp::osqp
)
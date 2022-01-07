cmake_minimum_required(VERSION 3.0.2)
project(rrt_planner)
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
  ${Eigen3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  include
  SYSTEM
)

add_library(${PROJECT_NAME}
  ${PROJECT_SOURCE_DIR}/src/rrt_planners.cpp
  ${PROJECT_SOURCE_DIR}/src/State/State.cpp
  ${PROJECT_SOURCE_DIR}/src/EuclideanSpace/EuclideanSpace.cpp
  ${PROJECT_SOURCE_DIR}/src/Constraint/ConstraintBase.cpp
  ${PROJECT_SOURCE_DIR}/src/Constraint/GridConstraint/GridConstraint.cpp
  ${PROJECT_SOURCE_DIR}/src/Constraint/PointCloudConstraint/PointCloudConstraint.cpp
  ${PROJECT_SOURCE_DIR}/src/Sampler/Sampler.cpp
  ${PROJECT_SOURCE_DIR}/src/Node/Node.cpp
  ${PROJECT_SOURCE_DIR}/src/Node/NodeListBase.cpp
  ${PROJECT_SOURCE_DIR}/src/Node/SimpleNodeList/SimpleNodeList.cpp
  ${PROJECT_SOURCE_DIR}/src/Node/KDTreeNodeList/KDTreeNodeList.cpp
  ${PROJECT_SOURCE_DIR}/src/Planner/PlannerBase.cpp
  ${PROJECT_SOURCE_DIR}/src/Planner/RRT/RRT.cpp
  ${PROJECT_SOURCE_DIR}/src/Planner/RRTStar/RRTStar.cpp
  ${PROJECT_SOURCE_DIR}/src/Planner/InformedRRTStar/InformedRRTStar.cpp
)
add_executable(${PROJECT_NAME}_node src/simple_rrt.cpp)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${EIGEN3_LIBS}
  probabilistic_roadmap
  map_server
  ${PROJECT_NAME}
)

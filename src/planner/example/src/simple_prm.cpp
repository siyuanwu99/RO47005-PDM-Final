#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Eigen>

#include "prm_planner.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "planner_example");
  ros::NodeHandle nh("~");
  PRM prm_planner(nh);
  ros::spin();

  return 0;
}

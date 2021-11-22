#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Eigen>

#include "prm_planner.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "planner_example");
  PRM prm_planner;
  // ros::Rate r(1);
  // while (ros::ok) {
  // prm_planner.planner_visualization();
    // r.sleep();
  // }
  ros::spin();

  return 0;
}

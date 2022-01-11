#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Eigen>

#include "prm_planner.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "planner");
  ros::NodeHandle nh("~");
  PRM prm_planner(nh);
  // ros::Rate r(0.1);
  
  // while (ros::ok()){
  //   prm_planner.rate_publisher();
  //   ros::spinOnce();
  //   r.sleep();
  // }
  ros::spin();
  return 0;
}

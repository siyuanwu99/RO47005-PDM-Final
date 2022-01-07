#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Eigen>

#include "rrt_planners.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "rrt_planner");
  ros::NodeHandle nh("~");
  RRTS rrt_planner(nh);
  // ros::Rate r(0.1);
  
  // while (ros::ok()){
  //   rrt_planner.rate_publisher();
  //   ros::spinOnce();
  //   r.sleep();
  // }
  ros::spin();
  return 0;
}

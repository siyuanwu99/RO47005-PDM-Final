#include <Eigen/Eigen>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include "prm_planner.h"
#include<visualization_msgs/Marker.h>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "planner_example");
  PRM prm_planner;
  ros::Rate r(20);
  while(ros::ok){
    prm_planner.planner_visualization();
    r.sleep();
  }
  
  return 0;
}

/**
 * @file test_traj_opt_node.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2021-12-07
 *
 * @copyright Copyright (c) 2021
 *
 */

#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <ros/ros.h>
#include <traj_opt/poly_opt.h>
#include <visualization_msgs/Marker.h>

using namespace traj_utils;

MiniSnapClosedForm* mini_snap;
PolyTraj* poly_traj;
ros::Subscriber waypoint_sub;
ros::Publisher trajectory_pub;

void waypointCallback(const geometry_msgs::PoseArray& wp) {
  std::vector<Eigen::Vector3d> waypoints;
  std::vector<double> time_allocations;
  waypoints.clear();

  for (int k = 0; k < (int)wp.poses.size(); k++) {
    Eigen::Vector3d pt(wp.poses[k].position.x, wp.poses[k].position.y,
                       wp.poses[k].position.z);
    std::cout << "Pos: " << pt(0) << " " << pt(1) << ' ' << pt(2) << std::endl;
    waypoints.push_back(pt);

    if (k > 0) {
      time_allocations.push_back(0.5);
    }
    if (wp.poses[k].position.z < 0.0) break;
  }

  double T = 0.5 * time_allocations.size();

  std::cout << "Time: " << time_allocations.size() << std::endl;

  mini_snap = new MiniSnapClosedForm(waypoints, time_allocations);
  poly_traj = new PolyTraj(time_allocations.size(), 7);

  mini_snap->initParams();
  mini_snap->solve(poly_traj);

  nav_msgs::Path path;
  double dt = 0.05;
  ros::Time current_time = ros::Time::now();

  path.header.frame_id = "world";
  path.header.stamp = current_time;

  for (double t = 0.0; t < T; t += dt) {
    geometry_msgs::PoseStamped point;
    point.header.frame_id = "world";
    point.header.stamp = current_time + ros::Duration(t);
    Eigen::Vector3d pos = poly_traj->getWayPoints(t);
    point.pose.position.x = pos(0);
    point.pose.position.y = pos(1);
    point.pose.position.z = pos(2);
    point.pose.orientation.w = 1;
    point.pose.orientation.x = 0;
    point.pose.orientation.y = 0;
    point.pose.orientation.z = 0;

    path.poses.push_back(point);
  }

  // trajectory_pub.publish(poin  ts);
  trajectory_pub.publish(path);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_traj_opt_node");
  ros::NodeHandle nh("~");

  ROS_INFO("Initialize node");
  waypoint_sub = nh.subscribe("waypoint", 100, waypointCallback);
  trajectory_pub =
      nh.advertise<nav_msgs::Path>("vis_waypoint_path", 1);

  // ros::Rate rate(10);
  // while (ros::ok()) {
  //   ros::spinOnce();
  //   rate.sleep();
  // }
  ros::spin();
  return 0;
}

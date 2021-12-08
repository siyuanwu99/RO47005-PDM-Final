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
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <traj_opt/poly_opt.h>
#include <visualization_msgs/Marker.h>

using namespace traj_utils;

MiniSnapClosedForm* mini_snap;
PolyTraj* poly_traj;
ros::Subscriber waypoint_sub;
ros::Publisher waypoints_pub;
ros::Publisher trajectory_pub;

void waypointCallback(const geometry_msgs::PoseArray& wp) {
  std::vector<Eigen::Vector3d> waypoints;
  std::vector<double> time_allocations;
  waypoints.clear();

  // estimated speed and time for every step
  double speed = 2.5, step_time = 0.5;

  for (int k = 0; k < (int)wp.poses.size(); k++) {
    Eigen::Vector3d pt(wp.poses[k].position.x, wp.poses[k].position.y,
                       wp.poses[k].position.z);
    // split line if it is out of scope given the estimated speed
    if (k > 0) {
      std::vector<double> diff = {pt(0) - waypoints.back()(0),
                                  pt(1) - waypoints.back()(1),
                                  pt(2) - waypoints.back()(2)};
      double dist = sqrt(pow(diff[0], 2) + pow(diff[1], 2) + pow(diff[2], 2));
      int split = dist / (speed * step_time);
      if (split == 0) split++;
      for (int i = 1; i <= split; i++) {
        Eigen::Vector3d mid_pt(waypoints.back()(0) + diff[0] / split * i,
                               waypoints.back()(1) + diff[1] / split * i,
                               waypoints.back()(2) + diff[2] / split * i);
        std::cout << "Pos: " << mid_pt(0) << " " << mid_pt(1) << ' '
                  << mid_pt(2) << std::endl;
        waypoints.push_back(mid_pt);
        time_allocations.push_back(0.5);
      }
    } else {
      std::cout << "Pos: " << pt(0) << " " << pt(1) << ' ' << pt(2)
                << std::endl;
      waypoints.push_back(pt);
    }
    if (wp.poses[k].position.z < 0.0) break;
  }

  double T = step_time * time_allocations.size();

  std::cout << "Time: " << time_allocations.size() << std::endl;

  mini_snap = new MiniSnapClosedForm(waypoints, time_allocations);
  poly_traj = new PolyTraj(time_allocations.size(), 7);

  mini_snap->initParams();
  mini_snap->solve(poly_traj);

  nav_msgs::Path path, wps_list;
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

  /* waypoints */
  wps_list.header.frame_id = "world";
  wps_list.header.stamp = current_time;
  for (auto i = 0; i < waypoints.size(); i++) {
    geometry_msgs::PoseStamped wps;
    wps.header.frame_id = "world";
    wps.header.stamp = current_time;
    wps.pose.position.x = waypoints[i](0);
    wps.pose.position.y = waypoints[i](1);
    wps.pose.position.z = waypoints[i](2);
    wps_list.poses.push_back(wps);
  }

  waypoints_pub.publish(wps_list);

  // trajectory_pub.publish(poin  ts);
  trajectory_pub.publish(path);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_traj_opt_node");
  ros::NodeHandle nh("~");

  ROS_INFO("Initialize node");
  waypoint_sub = nh.subscribe("waypoint", 100, waypointCallback);
  trajectory_pub = nh.advertise<nav_msgs::Path>("vis_waypoint_path", 1);
  waypoints_pub = nh.advertise<nav_msgs::Path>("vis_waypoint", 1);

  // ros::Rate rate(10);
  // while (ros::ok()) {
  //   ros::spinOnce();
  //   rate.sleep();
  // }
  ros::spin();
  return 0;
}

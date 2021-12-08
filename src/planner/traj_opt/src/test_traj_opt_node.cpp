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
#include <ros/ros.h>
#include <traj_opt/poly_opt.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>

using namespace traj_utils;

MiniSnapClosedForm* mini_snap;
PolyTraj* poly_traj;
ros::Subscriber waypoint_sub;
ros::Publisher trajectory_pub;

void waypointCallback(const nav_msgs::Path& wp) {
  std::vector<Eigen::Vector3d> waypoints;
  std::vector<double> time_allocations;
  waypoints.clear();

  for (int k = 0; k < (int)wp.poses.size(); k++) {
    Eigen::Vector3d pt(wp.poses[k].pose.position.x, wp.poses[k].pose.position.y,
                       wp.poses[k].pose.position.z);
    std::cout << "Pos: " << pt(0) << " " << pt(1) << ' ' << pt(2) << std::endl;
    waypoints.push_back(pt);

    if (k > 0) {
      time_allocations.push_back(0.5);
    }
    if (wp.poses[k].pose.position.z < 0.0) break;
  }

  double T = 0.5 * time_allocations.size();

  std::cout << "Time: " << time_allocations.size() << std::endl;

  mini_snap = new MiniSnapClosedForm(waypoints, time_allocations);
  poly_traj = new PolyTraj(time_allocations.size(), 7);

  mini_snap->initParams();
  mini_snap->solve(poly_traj);

  visualization_msgs::Marker points;
  points.header.frame_id = "map";
  points.header.stamp = ros::Time::now();
  points.ns = "wp_path";
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.pose.orientation.x = 0.0;
  points.pose.orientation.y = 0.0;
  points.pose.orientation.z = 0.0;

  int id = 0;
  points.id = id;
  points.type = visualization_msgs::Marker::SPHERE_LIST;

  points.scale.x = 0.3;
  points.scale.y = 0.3;
  points.scale.z = 0.3;
  points.color.a = 1.0;
  points.color.r = 0.0;
  points.color.g = 0.0;
  points.color.b = 0.0;

  for (double t = 0.0; t < T; t += 0.05) {
    geometry_msgs::Point p;
    Eigen::Vector3d pos = poly_traj->getWayPoints(t);
    p.x = pos(0);
    p.y = pos(1);
    p.z = pos(2);
    points.points.push_back(p);
  }

  trajectory_pub.publish(points);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_traj_opt_node");
  ros::NodeHandle nh("~");

  ROS_INFO("Initialize node");
  waypoint_sub = nh.subscribe("waypoint", 100, waypointCallback);
  trajectory_pub =
      nh.advertise<visualization_msgs::Marker>("vis_waypoint_path", 1);

  // ros::Rate rate(10);
  // while (ros::ok()) {
  //   ros::spinOnce();
  //   rate.sleep();
  // }
  ros::spin();
  return 0;
}

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
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <traj_opt/corridor_minisnap.h>
#include <traj_opt/poly_opt.h>
#include <visualization_msgs/Marker.h>

#include <chrono>

#include "decomp_ros_utils/data_ros_utils.h"
#include "decomp_util/ellipsoid_decomp.h"
#include "map_server/grid_map.h"

using namespace traj_utils;
using namespace traj_opt;

// MiniSnapClosedForm* mini_snap;
CorridorMiniSnap Chen_mini_snap_;
CorridorMiniSnapOriginal Millinger_mini_snap_;
Trajectory Chen_traj_;
Trajectory Millinger_traj_;
GridMap::Ptr map_ptr_;
vec_Vec3f observations;

static int traj_id_ = 0;
ros::Subscriber waypoint_sub;
ros::Subscriber map_sub;
ros::Publisher waypoints_pub;
ros::Publisher Mellinger_trajectory_pub;
ros::Publisher Chen_trajectory_pub;
ros::Publisher pos_cmd_pub;
ros::Publisher sfc_pub;

ros::Time traj_start_;
ros::Time traj_end_;

void visualizeCorridors(
    const std::vector<Eigen::Matrix<double, 6, -1>> hPolys) {
  decomp_ros_msgs::PolyhedronArray poly_msg;
  for (int i = 0; i < hPolys.size(); i++) {
    Eigen::MatrixXd hpoly = hPolys[i];
    decomp_ros_msgs::Polyhedron msg;
    for (int j = 0; j < hpoly.cols(); j++) {
      geometry_msgs::Point pt, n;
      pt.x = hpoly.col(j)(3);
      pt.y = hpoly.col(j)(4);
      pt.z = hpoly.col(j)(5);
      n.x = hpoly.col(j)(0);
      n.y = hpoly.col(j)(1);
      n.z = hpoly.col(j)(2);
      msg.points.push_back(pt);
      msg.normals.push_back(n);
    }
    poly_msg.polyhedrons.push_back(msg);
  }
  poly_msg.header.frame_id = "world";
  poly_msg.header.stamp = ros::Time::now();
  sfc_pub.publish(poly_msg);
}

std::vector<Eigen::Matrix<double, 6, -1>> polyhTypeConverter(
    vec_E<Polyhedron<3>> vs) {
  std::vector<Eigen::Matrix<double, 6, -1>> polys;
  polys.reserve(vs.size());
  for (const auto& v : vs) {
    Eigen::MatrixXd poly;
    poly.resize(6, v.hyperplanes().size());
    int i = 0;
    for (const auto& p : v.hyperplanes()) {
      poly.col(i).tail<3>() = p.p_;
      poly.col(i).head<3>() = p.n_;
      i++;
    }
    polys.push_back(poly);
  }
  return polys;
}

/**
 * @brief waypoints callback, calls when waypoints is received
 *
 * @param wp
 */
void waypointCallback(const geometry_msgs::PoseArray& wp) {
  std::vector<Eigen::Vector3d> waypoints;
  vec_Vec3f waypointsf;
  std::vector<double> time_allocations;
  waypoints.clear();

  // estimated speed and time for every step
  double speed = 2.5, step_time = 0.8;

  // read all waypoints from PRM graph
  for (int k = 0; k < (int)wp.poses.size(); k++) {
    Eigen::Vector3d pt(wp.poses[k].position.x, wp.poses[k].position.y,
                       wp.poses[k].position.z);
    Vec3f ptf(wp.poses[k].position.x, wp.poses[k].position.y,
              wp.poses[k].position.z);
    std::cout << "Pos: " << pt(0) << " " << pt(1) << ' ' << pt(2) << std::endl;
    waypoints.push_back(pt);
    waypointsf.push_back(ptf);
    if (k > 0) {
      time_allocations.push_back(step_time);
    }
  }
    // get total time
  time_allocations[0] = 2 * step_time;
  time_allocations.back() = 2 * step_time;
  double T = step_time * (time_allocations.size() + 2);
  std::cout << "Time: " << time_allocations.size() << std::endl;

  /* visualize waypoints */
  nav_msgs::Path wps_list;
  wps_list.header.frame_id = "world";
  wps_list.header.stamp = traj_start_;
  for (auto i = 0; i < waypoints.size(); i++) {
    geometry_msgs::PoseStamped wps;
    wps.header.frame_id = "world";
    wps.header.stamp = traj_start_;
    wps.pose.position.x = waypoints[i](0);
    wps.pose.position.y = waypoints[i](1);
    wps.pose.position.z = waypoints[i](2);
    wps_list.poses.push_back(wps);
  }
  waypoints_pub.publish(wps_list);

  /* get initial states and end states */
  Eigen::Vector3d zero(0.0, 0.0, 0.0);
  Eigen::Vector3d init_pos = waypoints[0];
  Eigen::Vector3d finl_pos = waypoints.back();
  Eigen::Matrix3d init_state;
  Eigen::Matrix3d finl_state;
  init_state << init_pos, zero, zero;
  finl_state << finl_pos, zero, zero;
  std::cout << "init\t" << init_pos.transpose() << std::endl;
  std::cout << "final\t" << finl_pos.transpose() << std::endl;

  /**
   * @brief generate flight corridors
   * Based on open-source codes from Sikang Liu
   */
  std::vector<Eigen::Matrix<double, 6, -1>> corridor;
  EllipsoidDecomp3D decomp_util;
  decomp_util.set_obs(observations);
  decomp_util.set_local_bbox(Vec3f(1, 2, 1));
  decomp_util.dilate(waypointsf);
  corridor = polyhTypeConverter(decomp_util.get_polyhedrons());
  /* clean buffer */
  ROS_INFO_STREAM("corridor size: " << corridor.size());
  visualizeCorridors(corridor);

  // initialize optimizer
  std::vector<Eigen::Vector3d> inter_waypoints(waypoints.begin() + 1,
                                               waypoints.end() - 1);

  // for (auto it = inter_waypoints.begin(); it != inter_waypoints.end(); ++it) {
  //   std::cout << "pos:\t" << it->transpose() << std::endl;
  // }
  std::cout << "Wpts: " << inter_waypoints.size() << std::endl;
  // Chen_mini_snap_.reset(init_state, finl_state, inter_waypoints,
  // time_allocations);
  std::chrono::high_resolution_clock::time_point tic =
      std::chrono::high_resolution_clock::now();

  Chen_mini_snap_.reset(init_state, finl_state, time_allocations, corridor);
  Chen_mini_snap_.optimize();
  Chen_mini_snap_.getTrajectory(&Chen_traj_);
  int I = 10;  // max iterations
  int i = 0;
  while (!Chen_mini_snap_.isCorridorSatisfied(Chen_traj_) && i++ < I) {
    std::cout << "out of corridor:\t" << i << std::endl;
    Chen_mini_snap_.reOptimize();
    Chen_mini_snap_.getTrajectory(&Chen_traj_);
  }
  // apply minimum snap optimization

  std::chrono::high_resolution_clock::time_point toc =
      std::chrono::high_resolution_clock::now();
  double compTime =
      std::chrono::duration_cast<std::chrono::microseconds>(toc - tic).count() *
      1.0e-3;
  std::cout << "total iterations: " << i << std::endl;
  std::cout << std::chrono::duration_cast<std::chrono::microseconds>(toc - tic)
                       .count() *
                   1.0e-3
            << "ms" << std::endl;
  std::cout << "Max Vel Rate: " << Chen_traj_.getMaxVelRate() << std::endl;
  std::cout << "Total Time: " << Chen_traj_.getDuration() << std::endl;

  traj_id_++;
  std::cout << "\033[42m"
            << "Get new trajectory:\tidx: " << traj_id_ << "\033[0m"
            << std::endl;

  /* millinger mini snap */
  tic = std::chrono::high_resolution_clock::now();
  Millinger_mini_snap_.reset(init_state, finl_state, time_allocations, corridor);
  Millinger_mini_snap_.optimize();
  Millinger_mini_snap_.getTrajectory(&Millinger_traj_);
  toc = std::chrono::high_resolution_clock::now();
  std::cout << std::chrono::duration_cast<std::chrono::microseconds>(toc - tic)
                       .count() * 1.0e-3
            << "ms" << std::endl;
  std::cout << "Max Vel Rate: " << Millinger_traj_.getMaxVelRate() << std::endl;
  std::cout << "Total Time: " << Millinger_traj_.getDuration() << std::endl;

  // initialize visualization
  nav_msgs::Path chen_path, mellinger_path;
  double dt = 0.05;
  traj_start_ = ros::Time::now();              // start timestamp
  traj_end_ = traj_start_ + ros::Duration(T);  // end timestamp

  chen_path.header.frame_id = "world";
  chen_path.header.stamp = traj_start_;

  for (double t = 0.0; t < T; t += dt) {
    geometry_msgs::PoseStamped point;
    point.header.frame_id = "world";
    point.header.stamp = traj_start_ + ros::Duration(t);

    Eigen::Vector3d pos = Chen_traj_.getPos(t);
    point.pose.position.x = pos(0);
    point.pose.position.y = pos(1);
    point.pose.position.z = pos(2);
    point.pose.orientation.w = 1;
    point.pose.orientation.x = 0;
    point.pose.orientation.y = 0;
    point.pose.orientation.z = 0;
    chen_path.poses.push_back(point);

    geometry_msgs::PoseStamped point2;
    point2.header.frame_id = "world";
    point2.header.stamp = traj_start_ + ros::Duration(t);
    Eigen::Vector3d pos2 = Millinger_traj_.getPos(t);
    point2.pose.position.x = pos2(0);
    point2.pose.position.y = pos2(1);
    point2.pose.position.z = pos2(2);
    point2.pose.orientation.w = 1;
    point2.pose.orientation.x = 0;
    point2.pose.orientation.y = 0;
    point2.pose.orientation.z = 0;
    mellinger_path.poses.push_back(point2);
  }

  Mellinger_trajectory_pub.publish(mellinger_path);
  Chen_trajectory_pub.publish(chen_path);

  corridor.clear();
}

/**
 * @brief callback, calls every 10ms, send commands to controller
 *
 * @param te
 */
void commandCallback(const ros::TimerEvent& te) {
  if (traj_id_ < 1) {
    return;
  }
  ros::Time now = ros::Time::now();
  quadrotor_msgs::PositionCommand pos_cmd;

  pos_cmd.header.stamp = now;
  pos_cmd.header.frame_id = "world";
  pos_cmd.trajectory_id = traj_id_;

  double t = ros::Duration(now - traj_start_).toSec();

  /* get position commands */
  Eigen::Vector3d pos = Chen_traj_.getPos(t);
  pos_cmd.position.x = pos(0);
  pos_cmd.position.y = pos(1);
  pos_cmd.position.z = pos(2);

  /* get velocity commands */
  Eigen::Vector3d vel = Chen_traj_.getVel(t);
  pos_cmd.velocity.x = vel(0);
  pos_cmd.velocity.y = vel(1);
  pos_cmd.velocity.z = vel(2);

  /*get acceleration commands */
  Eigen::Vector3d acc = Chen_traj_.getAcc(t);
  pos_cmd.acceleration.x = acc(0);
  pos_cmd.acceleration.y = acc(1);
  pos_cmd.acceleration.z = acc(2);

  // pos_cmd.yaw = 0;
  // pos_cmd.yaw_dot = 0;

  pos_cmd_pub.publish(pos_cmd);
}

void mapCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  map_ptr_->initFromPointCloud(msg);
  // map_ptr_->publish();
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*msg, cloud);
  observations.resize(cloud.points.size());
  int idx = 0;
  for (auto it = cloud.begin(); it != cloud.end(); ++it) {
    Eigen::Vector3f p = it->getVector3fMap();
    observations[idx](0) = p(0);
    observations[idx](1) = p(1);
    observations[idx](2) = p(2);
    idx++;
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_traj_opt_node");
  ros::NodeHandle nh("~");

  map_ptr_.reset(new GridMap());
  map_ptr_->initGridMap(nh);

  ROS_INFO("Initialize node");
  waypoint_sub = nh.subscribe("waypoint", 100, waypointCallback);
  map_sub = nh.subscribe("map", 1, mapCallback);
  Chen_trajectory_pub = nh.advertise<nav_msgs::Path>("chen_trajectory", 1);
  Mellinger_trajectory_pub = nh.advertise<nav_msgs::Path>("mellinger_trajectory", 1);
  waypoints_pub = nh.advertise<nav_msgs::Path>("vis_waypoint", 1);
  pos_cmd_pub =
      nh.advertise<quadrotor_msgs::PositionCommand>("position_command", 10);
  sfc_pub = nh.advertise<decomp_ros_msgs::PolyhedronArray>("safe_corridor", 1);
  // publish quadrotor commands every 10ms
  ros::Timer command_timer =
      nh.createTimer(ros::Duration(0.01), commandCallback);

  ros::spin();
  return 0;
}

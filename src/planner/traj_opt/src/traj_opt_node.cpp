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

#include "decomp_ros_utils/data_ros_utils.h"
#include "decomp_util/ellipsoid_decomp.h"
#include "map_server/grid_map.h"

using namespace traj_utils;
using namespace traj_opt;

// MiniSnapClosedForm* mini_snap;
// CorridorMiniSnap mini_snap_;
MiniSnap mini_snap_;
Trajectory traj_;
GridMap::Ptr map_ptr_;
vec_Vec3f observations;

static int traj_id_ = 0;
ros::Subscriber waypoint_sub;
ros::Subscriber map_sub;
ros::Publisher waypoints_pub;
ros::Publisher trajectory_pub;
ros::Publisher pos_cmd_pub;
ros::Publisher sfc_pub;

ros::Time traj_start_;
ros::Time traj_end_;

void visualizeCorridors(
    const std::vector<Eigen::Matrix<double, 6, -1>> hPolys) {
  decomp_ros_msgs::PolyhedronArray poly_msg;
  for (int i=0; i < hPolys.size(); i++) {
    Eigen::MatrixXd hpoly = hPolys[i];
    decomp_ros_msgs::Polyhedron msg;
    for (int j=0; j < hpoly.cols(); j++) {
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
  double speed = 2.5, step_time = 0.5;

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
      time_allocations.push_back(0.5);
    }
  }

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
  
  // get total time
  time_allocations[0] = 1;
  time_allocations.back() = 1;
  double T = step_time * time_allocations.size() + 1;
  std::cout << "Time: " << time_allocations.size() << std::endl;

  // initialize optimizer
  // mini_snap_.reset(init_state, finl_state, time_allocations, corridor);
  std::vector<Eigen::Vector3d> inter_waypoints(waypoints.begin() + 1,
                                               waypoints.end() - 1);

  for (auto it=inter_waypoints.begin(); it!=inter_waypoints.end(); ++it) {
    std::cout << "pos:\t" << it->transpose() << std::endl;
  }
  std::cout << "Wpts: " << inter_waypoints.size() << std::endl;
  mini_snap_.reset(init_state, finl_state, inter_waypoints, time_allocations);
  mini_snap_.optimize();
  mini_snap_.getTrajectory(&traj_);

  int I = 10;  // max iterations
  int i = 0;
  // while (!mini_snap_.isCorridorSatisfied(traj_) && i++ < I) {
  //   std::out << "out of corridor" << std::endl;
  //   mini_snap_.reOptimize();
  //   mini_snap_.getTrajectory(&traj_);
  // }
  // apply minimum snap optimization
  traj_id_++;

  std::cout << "\033[42m"
            << "Get new trajectory:\tidx: " << traj_id_ << "\033[0m"
            << std::endl;

  // initialize visualization
  nav_msgs::Path path, wps_list;
  double dt = 0.05;
  traj_start_ = ros::Time::now();              // start timestamp
  traj_end_ = traj_start_ + ros::Duration(T);  // end timestamp

  path.header.frame_id = "world";
  path.header.stamp = traj_start_;

  for (double t = 0.0; t < T; t += dt) {
    geometry_msgs::PoseStamped point;
    point.header.frame_id = "world";
    point.header.stamp = traj_start_ + ros::Duration(t);
    Eigen::Vector3d pos = traj_.getPos(t);
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
  trajectory_pub.publish(path);

  /* clean buffer */
  ROS_INFO_STREAM("corridor size: " << corridor.size());
  visualizeCorridors(corridor);
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
  Eigen::Vector3d pos = traj_.getPos(t);
  pos_cmd.position.x = pos(0);
  pos_cmd.position.y = pos(1);
  pos_cmd.position.z = pos(2);

  // /* get velocity commands */
  // Eigen::Vector3d vel = traj_.getVel(t);
  // pos_cmd.velocity.x = vel(0);
  // pos_cmd.velocity.y = vel(1);
  // pos_cmd.velocity.z = vel(2);

  // /*get acceleration commands */
  // Eigen::Vector3d acc = traj_.getAcc(t);
  // pos_cmd.acceleration.x = acc(0);
  // pos_cmd.acceleration.y = acc(1);
  // pos_cmd.acceleration.z = acc(2);

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
  trajectory_pub = nh.advertise<nav_msgs::Path>("vis_waypoint_path", 1);
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

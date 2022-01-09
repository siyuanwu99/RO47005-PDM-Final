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
CorridorMiniSnap mini_snap_;
// MiniSnap mini_snap_;
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
ros::Publisher color_vel_pub;

ros::Time traj_start_;
ros::Time traj_end_;

double speed;

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

inline Eigen::Vector3d jetColor(double a) {
  double s = a * 4;
  Eigen::Vector3d c;  // [r, g, b]
  switch ((int)floor(s)) {
    case 0:
      c << 0, 0, s;
      break;
    case 1:
      c << 0, s - 1, 1;
      break;
    case 2:
      c << s - 2, 1, 3 - s;
      break;
    case 3:
      c << 1, 4 - s, 0;
      break;
    default:
      c << 1, 0, 0;
      break;
  }
  return c;
}

void visualizeTraj(const Trajectory& appliedTraj, double maxV) {
  visualization_msgs::Marker traj_marker;
  traj_marker.header.frame_id = "world";
  traj_marker.header.stamp = ros::Time::now();
  traj_marker.type = visualization_msgs::Marker::LINE_LIST;
  traj_marker.pose.orientation.w = 1.00;
  traj_marker.action = visualization_msgs::Marker::ADD;
  traj_marker.id = 0;
  traj_marker.ns = "trajectory";
  traj_marker.color.r = 0.00;
  traj_marker.color.g = 0.50;
  traj_marker.color.b = 1.00;
  traj_marker.scale.x = 0.10;

  double T = 0.05;
  Eigen::Vector3d lastX = appliedTraj.getPos(0.0);
  for (double t = T; t < appliedTraj.getDuration(); t += T) {
    std_msgs::ColorRGBA c;
    Eigen::Vector3d jets = jetColor(appliedTraj.getVel(t).norm() / maxV);
    c.r = jets[0];
    c.g = jets[1];
    c.b = jets[2];
    c.a = 0.8;

    geometry_msgs::Point point;
    Eigen::Vector3d X = appliedTraj.getPos(t);
    point.x = lastX(0);
    point.y = lastX(1);
    point.z = lastX(2);
    traj_marker.points.push_back(point);
    traj_marker.colors.push_back(c);
    point.x = X(0);
    point.y = X(1);
    point.z = X(2);
    traj_marker.points.push_back(point);
    traj_marker.colors.push_back(c);
    lastX = X;
  }
  color_vel_pub.publish(traj_marker);
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
    if (k > 0) {
      Eigen::Vector3d pre_pt(waypoints.back()(0), waypoints.back()(1),
                             waypoints.back()(2));
      double dist = (pt - pre_pt).norm();
      time_allocations.push_back(dist / speed);
      std::cout << "t: " << time_allocations.back() << std::endl;
    }

    Vec3f ptf(wp.poses[k].position.x, wp.poses[k].position.y,
              wp.poses[k].position.z);
    std::cout << "Pos: " << pt(0) << " " << pt(1) << ' ' << pt(2) << std::endl;
    waypoints.push_back(pt);
    waypointsf.push_back(ptf);
  }

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

  // get total time  // trapezoidal speed curve
  time_allocations[0] = 2 * time_allocations[0];
  time_allocations.back() *= 2;
  double T = 0;
  for (auto it = time_allocations.begin(); it != time_allocations.end(); ++it) {
    T += (*it);
  }
  std::cout << "Time Size: " << time_allocations.size() << std::endl;
  std::cout << "Time: " << T << std::endl;

  // initialize optimizer
  std::vector<Eigen::Vector3d> inter_waypoints(waypoints.begin() + 1,
                                               waypoints.end() - 1);

  // for (auto it = inter_waypoints.begin(); it != inter_waypoints.end(); ++it)
  // {
  //   std::cout << "pos:\t" << it->transpose() << std::endl;
  // }
  // std::cout << "Wpts: " << inter_waypoints.size() << std::endl;
  // mini_snap_.reset(init_state, finl_state, inter_waypoints,
  // time_allocations);
  std::chrono::high_resolution_clock::time_point tic =
      std::chrono::high_resolution_clock::now();

  mini_snap_.reset(init_state, finl_state, time_allocations, corridor);
  mini_snap_.optimize();
  mini_snap_.getTrajectory(&traj_);
  int I = 10;  // max iterations
  int i = 0;
  while (!mini_snap_.isCorridorSatisfied(traj_) && i++ < I) {
    std::cout << "out of corridor:\t" << i << std::endl;
    mini_snap_.reOptimize();
    mini_snap_.getTrajectory(&traj_);
  }
  // apply minimum snap optimization

  std::chrono::high_resolution_clock::time_point toc =
      std::chrono::high_resolution_clock::now();
  double compTime =
      std::chrono::duration_cast<std::chrono::microseconds>(toc - tic).count() *
      1.0e-3;
  std::cout << "\033[35m EVALUATIONS" << std::endl;
  std::cout << " number of pieces: " << time_allocations.size() << std::endl;
  std::cout << "[TrajOpt] Iterations: " << i << std::endl;
  std::cout << "[TrajOpt] Computation time: "
            << 1.0e-3 * std::chrono::duration_cast<std::chrono::microseconds>(
                            toc - tic)
                            .count()
            << "ms" << std::endl;
  std::cout << "[TrajOpt] Final cost: " << mini_snap_.getMinimumCost()
            << std::endl;
  std::cout << "[TrajOpt] Max velocity: " << traj_.getMaxVelRate() << std::endl;
  std::cout << "[TrajOpt] Max acclerate: " << traj_.getMaxAccRate()
            << std::endl;
  std::cout << "[TrajOpt] Total time: " << traj_.getDuration() << std::endl;
  std::cout << " \033[0m" << std::endl;

  traj_id_++;

  // initialize visualization
  nav_msgs::Path path;
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
  trajectory_pub.publish(path);

  visualizeTraj(traj_, 5.0);
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

  /* get velocity commands */
  Eigen::Vector3d vel = traj_.getVel(t);
  pos_cmd.velocity.x = vel(0);
  pos_cmd.velocity.y = vel(1);
  pos_cmd.velocity.z = vel(2);

  /*get acceleration commands */
  Eigen::Vector3d acc = traj_.getAcc(t);
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
  /* get parameters */
  if (nh.getParam("speed", speed)) {
    ROS_INFO("[TrajOpt] get speed: %f", speed);
  } else {
    speed = 2.0;
    ROS_INFO("[TrajOpt] use default speed: %f", speed);
  }

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
  color_vel_pub =
      nh.advertise<visualization_msgs::Marker>("colored_trajectory", 1);

  // publish quadrotor commands every 10ms
  ros::Timer command_timer =
      nh.createTimer(ros::Duration(0.01), commandCallback);

  ros::spin();
  return 0;
}

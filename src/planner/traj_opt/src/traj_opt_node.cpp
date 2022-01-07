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
#include <traj_opt/poly_opt.h>
#include <visualization_msgs/Marker.h>

#include "decomp_ros_utils/data_ros_utils.h"
#include "decomp_util/ellipsoid_decomp.h"
#include "flight_corridor/safe_flight_corridor.h"
#include "map_server/grid_map.h"

using namespace traj_utils;

MiniSnapClosedForm* mini_snap;
PolyTraj* poly_traj_;
FlightCorridor sfc;
GridMap::Ptr map_ptr_;

static int traj_id_ = 0;
ros::Subscriber waypoint_sub;
ros::Subscriber map_sub;
ros::Publisher waypoints_pub;
ros::Publisher trajectory_pub;
ros::Publisher pos_cmd_pub;
ros::Publisher sfc_pub;

ros::Time traj_start_;
ros::Time traj_end_;

void visualizeCorridors(const std::vector<Eigen::Matrix<double, 6, 6>> hPolys) {
  vec_E<Polyhedron3D> polyhedra;
  polyhedra.reserve(hPolys.size());
  for (const auto& ele : hPolys) {
    Polyhedron3D hPoly;
    for (int i = 0; i < ele.cols(); i++) {
      hPoly.add(Hyperplane3D(ele.col(i).tail<3>(), ele.col(i).head<3>()));
    }
    polyhedra.push_back(hPoly);
  }
  decomp_ros_msgs::PolyhedronArray poly_msg =
      DecompROS::polyhedron_array_to_ros(polyhedra);
  poly_msg.header.frame_id = "world";
  poly_msg.header.stamp = ros::Time::now();
  sfc_pub.publish(poly_msg);
}

/**
 * @brief waypoints callback, calls when waypoints is received
 *
 * @param wp
 */
void waypointCallback(const geometry_msgs::PoseArray& wp) {
  std::vector<Eigen::Vector3d> waypoints;
  std::vector<double> time_allocations;
  waypoints.clear();

  // estimated speed and time for every step
  double speed = 2.5, step_time = 0.5;

  // read all waypoints from PRM graph
  // for (int k = 0; k < (int)wp.poses.size(); k++) {
  //   Eigen::Vector3d pt(wp.poses[k].position.x, wp.poses[k].position.y,
  //                      wp.poses[k].position.z);
  //   // split line if it is out of scope given the estimated speed
  //   if (k > 0) {
  //     Eigen::Vector3d pre_pt(waypoints.back()(0), waypoints.back()(1),
  //                            waypoints.back()(2));
  //     std::vector<double> diff = {pt(0) - pre_pt(0), pt(1) - pre_pt(1),
  //                                 pt(2) - pre_pt(2)};
  //     double dist = sqrt(pow(diff[0], 2) + pow(diff[1], 2) + pow(diff[2], 2));
  //     int split = dist / (speed * step_time);
  //     if (split == 0) split++;

  //     for (int i = 1; i <= split; i++) {
  //       Eigen::Vector3d mid_pt(pre_pt(0) + diff[0] / split * i,
  //                              pre_pt(1) + diff[1] / split * i,
  //                              pre_pt(2) + diff[2] / split * i);
  //       std::cout << "Pos: " << mid_pt(0) << " " << mid_pt(1) << ' '
  //                 << mid_pt(2) << std::endl;
  //       waypoints.push_back(mid_pt);
  //       time_allocations.push_back(0.5);
  //     }
  //   } else {
  //     std::cout << "Pos: " << pt(0) << " " << pt(1) << ' ' << pt(2)
  //               << std::endl;
  //     waypoints.push_back(pt);
  //   }
  //   if (wp.poses[k].position.z < 0.0) break;
  // }
  for (int k = 0; k < (int)wp.poses.size(); k++) {
    Eigen::Vector3d pt(wp.poses[k].position.x, wp.poses[k].position.y,
                       wp.poses[k].position.z);
    std::cout << "Pos: " << pt(0) << " " << pt(1) << ' ' << pt(2) << std::endl;
    waypoints.push_back(pt);

    if (k > 0) {
      time_allocations.push_back(0.5);
    }
  }

  // get safe flight corridor
  sfc.set_map(map_ptr_);
  if (!sfc.generate(waypoints)) {
    ROS_ERROR("Failed to generate flight corridors");
  }
  std::vector<Eigen::Matrix<double, 6, 6>> corridor;
  sfc.corridor2mat(corridor);

  // get total time
  time_allocations[0] = 1;
  time_allocations.back() = 1;
  double T = step_time * time_allocations.size() + 1;
  std::cout << "Time: " << time_allocations.size() << std::endl;

  // initialize optimizer
  mini_snap = new MiniSnapClosedForm(waypoints, time_allocations);
  poly_traj_ = new PolyTraj(time_allocations.size(), 7);

  // apply minimum snap optimization
  mini_snap->initParams();
  mini_snap->solve(poly_traj_);
  traj_id_++;

  std::cout << "\033[42m"
            << "Get new trajectory:\tidx: " << traj_id_ << "\033[0m"
            << std::endl;

  // initialize visualization
  nav_msgs::Path path, wps_list;
  double dt = 0.05;
  traj_start_ = ros::Time::now();
  traj_end_ = traj_start_ + ros::Duration(T);

  path.header.frame_id = "world";
  path.header.stamp = traj_start_;

  for (double t = 0.0; t < T; t += dt) {
    geometry_msgs::PoseStamped point;
    point.header.frame_id = "world";
    point.header.stamp = traj_start_ + ros::Duration(t);
    Eigen::Vector3d pos = poly_traj_->getWayPoints(t);
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

  // clean
  ROS_INFO_STREAM("corridor size: " << corridor.size());
  visualizeCorridors(corridor);
  sfc.cubes_.clear();
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
  Eigen::Vector3d pos = poly_traj_->getWayPoints(t);
  pos_cmd.position.x = pos(0);
  pos_cmd.position.y = pos(1);
  pos_cmd.position.z = pos(2);

  // /* get velocity commands */
  Eigen::Vector3d vel = poly_traj_->getVelocities(t);
  pos_cmd.velocity.x = vel(0);
  pos_cmd.velocity.y = vel(1);
  pos_cmd.velocity.z = vel(2);

  // /*get acceleration commands */
  Eigen::Vector3d acc = poly_traj_->getAcclections(t);
  pos_cmd.acceleration.x = acc(0);
  pos_cmd.acceleration.y = acc(1);
  pos_cmd.acceleration.z = acc(2);

  // pos_cmd.yaw = 0;
  // pos_cmd.yaw_dot = 0;

  pos_cmd_pub.publish(pos_cmd);
}

void mapCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  map_ptr_->initFromPointCloud(msg);
  map_ptr_->publish();
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

/**
 * @file rrt_planner.cpp
 * @author Ranbao Deng, Yangchen Sui
 * @brief 
 * @version 1.0
 * @date 2021-12-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "rrt_planners.h"

#include <stdlib.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <sstream>
#include <list>
#include <algorithm>
#include <queue>
#include <numeric>
#include <functional>

using std::vector;
using std::cout;
using std::endl;
namespace pln = planner;

//-------------------------------
//member function for RRTS planner
//-------------------------------
/**
 * @brief Construct a new RRTS::RRTS object
 * @author medalotte, Ranbao Deng
 */
RRTS::RRTS(const ros::NodeHandle & nh) {

    nh_ = nh;

    grid_map_ptr_.reset(new GridMap);
    grid_map_ptr_->initGridMap(nh_);
    pnt_cld_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
      "cloud_in", 1, &RRTS::pointCloudCallback, this);
    sub_ = nh_.subscribe("/move_base_simple/goal", 5, &RRTS::callback, this);
    odom_sub_ = nh_.subscribe("odometry", 5, &RRTS::OdomCallback, this);
    edge_pub_ = nh_.advertise<visualization_msgs::Marker>("edge_marker", 10);
    // path_pub_ = nh_.advertise<visualization_msgs::Marker>("path_marker", 10);
    node_pub_ = nh_.advertise<visualization_msgs::Marker>("node_markers", 10);
    path_raw_pub_ = nh_.advertise<geometry_msgs::PoseArray>("raw_path", 10);
    get_map_param();
}

/**
 * @brief get map parameters from parameter server(size, number of samples)
 * @author Moji Shi
 */
void RRTS::get_map_param() {
  if (nh_.getParam("/random_forest/map/x_size", map_size_x)) {
    ROS_INFO("get map x: %f", map_size_x);
  }
  if (nh_.getParam("/random_forest/map/y_size", map_size_y)) {
    ROS_INFO("get map y: %f", map_size_y);
  }
  if (nh_.getParam("/random_forest/map/z_size", map_size_z)) {
    ROS_INFO("get map z: %f", map_size_z);
  }
}

/**
 * @brief call back function of subscriber receiving topic "/move_base_simple/goal"
 * @author Moji Shi, Ranbao Deng
 * 
 * @param msg 
 */
void RRTS::callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {

    // definition of start and goal state
    current_pos_ = {0,0,0};
    pln::State start(current_pos_(0), current_pos_(1), current_pos_(2));
    ROS_INFO("Current position: %f, %f, %f",current_pos_(0),current_pos_(1),current_pos_(2));
    pln::State goal(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    ROS_INFO("Goal received: %f, %f, %f",msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);

    for (auto i=0;i<100;i++){
      // solve
      auto start_time = std::chrono::system_clock::now();
      bool status = planner->solve(start, goal);
      auto end_time = std::chrono::system_clock::now();
      ROS_INFO_STREAM("elapsed time : " << std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count() / 1000.0 << "ms");
      if(status) {
          auto node_list = planner->getNodeList();
          auto result = planner->getResult();

          // Visualization of edges and nodes
          auto leafs = node_list->searchLeafs();
          visualization_msgs::Marker line_list;
          line_list.header.frame_id = "world";
          line_list.header.stamp = ros::Time::now();
          line_list.ns = "planner";
          line_list.action = visualization_msgs::Marker::ADD;
          line_list.pose.orientation.w = 1.0;
          line_list.id = 2;
          line_list.type = visualization_msgs::Marker::LINE_LIST;
          line_list.scale.x = 0.05;
          vector<double> color = {1,0,0};
          line_list.color.r = color[0];
          line_list.color.g = color[1];
          line_list.color.b = color[2];
          line_list.color.a = 0.8;
          visualization_msgs::Marker points_;
          points_.header.frame_id = "world";
          points_.header.stamp = ros::Time::now();
          points_.ns = "planner";
          points_.action = visualization_msgs::Marker::ADD;
          points_.pose.orientation.w = 1.0;
          points_.id = 0;
          points_.type = visualization_msgs::Marker::POINTS;
          points_.scale.x = 0.2;
          points_.scale.y = 0.2;
          points_.color.g = 1.0f;
          points_.color.a = 1.0;
          for (auto node : leafs) {
              while (node->parent != nullptr) {                
                  geometry_msgs::Point p1,p2;
                  p1.x = node->state.vals[0];
                  p1.y = node->state.vals[1];
                  p1.z = node->state.vals[2];
                  p2.x = node->parent->state.vals[0];
                  p2.y = node->parent->state.vals[1];
                  p2.z = node->parent->state.vals[2];
                  line_list.points.push_back(p1);
                  line_list.points.push_back(p2);
                  points_.points.push_back(p1);
                  node = node->parent;
              }
          }
          edge_pub_.publish(line_list);
          node_pub_.publish(points_);

          //pass path to traj optimization
          geometry_msgs::PoseArray raw_path;
          raw_path.header.frame_id = "world";
          raw_path.header.stamp = ros::Time::now();
          for (const auto &r : result) {
              geometry_msgs::Pose point;
              point.position.x = r.vals[0];
              point.position.y = r.vals[1];
              point.position.z = r.vals[2];
              raw_path.poses.push_back(point);
          }
          path_raw_pub_.publish(raw_path);
          ROS_INFO_STREAM("Total cost : " << planner->getResultCost());
      }
      else {
          ROS_INFO("Could not find path!");
      }       
    }
}

void RRTS::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);
    int cloud_size = cloud.points.size();
    ROS_INFO("Received point clouds");
    grid_map_ptr_->pointCloudCallback(msg);
    // Initialization of Planner
    pln::EuclideanSpace space(3);
    std::vector<pln::Bound> bounds{pln::Bound(-map_size_x/2, map_size_x/2), pln::Bound(-map_size_y/2, map_size_y/2), pln::Bound(-map_size_z/2, map_size_z/2)};
    space.setBound(bounds);

    // definition of obstacle (point cloud type)
    pcl::PointXYZ pt;
    std::vector<pln::PointCloudConstraint::Hypersphere> obstacles;
    for (size_t i = 0; i < cloud_size; i++) {
        pt = cloud.points[i];
        obstacles.emplace_back(pln::State(pt.x, pt.y, pt.z), 0.2); // inflate
    }

    // definition of constraint using std::shared_ptr
    auto constraint = std::make_shared<pln::PointCloudConstraint>(space, obstacles);

    // planner init
    // planner = std::make_unique<pln::RRT>(
    //               3, 10000, 0.025, // DIM, max samples, sample rate
    //               3.0); // expand dist
    // planner = std::make_unique<pln::RRTStar>(
    //               3, 10000, 0.25, // DIM, max samples, sample rate
    //               50, 250); // expand dist, R
    planner = std::make_unique<pln::InformedRRTStar>(
                  3, 10000, 0.25, // DIM, max samples, sample rate
                  50, 250, // expand dist, R
                  5.0); // goal region radius

    // set constraint
    planner->setProblemDefinition(constraint);
    planner->setTerminateSearchCost(1330);

    ROS_INFO("RRT INITIALIZED");
}

void RRTS::OdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_pos_(0) = msg->pose.pose.position.x;
    current_pos_(1) = msg->pose.pose.position.y;
    current_pos_(2) = msg->pose.pose.position.z;
}
#include "prm_planner.h"

#include <stdlib.h>
#include <visualization_msgs/Marker.h>
#include<iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <sstream>
#include <list>
#include <algorithm>

using std::vector;
using std::cout;
using std::endl;

//member function for graph implementation

void Graph::insertVex(Vertice vex1){

    VexList.push_back(vex1);
    numVex++;

}
void Graph::insertEdge(const Vertice & vex1, const Vertice & vex2, int cost){

    //find vertexs in list
    auto itr1 = find(VexList.begin(), VexList.end(), vex1);
    auto itr2 = find(VexList.begin(), VexList.end(), vex2);
    
    if(itr1==VexList.end() || itr2==VexList.end())cout<<"insert edge failed, cannot find vertex"<<endl;
    else{

        //create new edge
        Edge * new_edge1 = new Edge(distance(VexList.begin(), itr1), distance(VexList.begin(), itr2), cost);
        Edge * new_edge2 = new Edge(distance(VexList.begin(), itr1), distance(VexList.begin(), itr2), cost);
        numEdge++;

        //add created edge to edge list stored in vertex
        if(itr1->FirstAdjacentEdge==nullptr)itr1->FirstAdjacentEdge = new_edge1;
        else{
            Edge* ptr = itr1->FirstAdjacentEdge;
            while(ptr->next!=nullptr)ptr = ptr->next;
            ptr->next = new_edge1;
        }
        if(itr2->FirstAdjacentEdge==nullptr)itr2->FirstAdjacentEdge = new_edge2;
        else{
            Edge* ptr = itr2->FirstAdjacentEdge;
            while(ptr->next!=nullptr)ptr = ptr->next;
            ptr->next = new_edge2;
        }
    }
}

PRM::PRM() {
  sub_ = nh_.subscribe("/move_base_simple/goal", 5, &PRM::callback, this);
  pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  prm_pub_ = nh_.advertise<visualization_msgs::Marker>("prm_markers", 10);

  get_map_param();
//   node_visualization();
}

void PRM::get_map_param() {
  if (nh_.getParam("/random_forest/map/x_size", map_size_x)) {
    ROS_INFO("get map x: %f", map_size_x);
  }
  if (nh_.getParam("/random_forest/map/y_size", map_size_y)) {
    ROS_INFO("get map y: %f", map_size_y);
  }
  if (nh_.getParam("/random_forest/map/z_size", map_size_z)) {
    ROS_INFO("get map z: %f", map_size_z);
  }
  if (nh_.getParam("/prm_planner/number_sample", n_sample)) {
    ROS_INFO("get sample number: %i", n_sample);
  }
}

// Visualization sample nodes
void PRM::node_visualization() {
  // generate random node
  visualization_msgs::Marker points_;
  points_.header.frame_id = "world";
  points_.header.stamp = ros::Time::now();
  points_.ns = "planner_example";
  points_.action = visualization_msgs::Marker::ADD;
  points_.pose.orientation.w = 1.0;
  points_.id = 0;
  points_.type = visualization_msgs::Marker::POINTS;
  points_.scale.x = 0.2;
  points_.scale.y = 0.2;
  points_.color.g = 1.0f;
  points_.color.a = 1.0;
  for (int i = 0; i < n_sample; i++) {
    geometry_msgs::Point p;
    p.x = ((double)rand() / (RAND_MAX)-0.5) * map_size_x;
    p.y = ((double)rand() / (RAND_MAX)-0.5) * map_size_y;
    p.z = ((double)rand() / (RAND_MAX)) * map_size_z;
    ROS_INFO("X:%f  Y:%f  Z:%f", p.x, p.y, p.z);
    points_.points.push_back(p);
  }
  prm_pub_.publish(points_);
}

void PRM::callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  this->start.x = 0;
  this->start.y = 0;
  this->start.z = 0;
  this->goal = msg->pose.position;
  PRM::planner_visualization();
  node_visualization();
}

void PRM::planner_visualization() {
  visualization_msgs::Marker line_strip;
  line_strip.header.frame_id = "world";
  line_strip.header.stamp = ros::Time::now();
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip.action = visualization_msgs::Marker::ADD;
  line_strip.ns = "planner_example";
  line_strip.pose.orientation.w = 1.0;
  line_strip.scale.x = 0.1;
  line_strip.color.b = 1.0;
  line_strip.color.a = 1.0;
  line_strip.id = 1;
  line_strip.points.push_back(start);
  line_strip.points.push_back(goal);
  pub_.publish(line_strip);
}

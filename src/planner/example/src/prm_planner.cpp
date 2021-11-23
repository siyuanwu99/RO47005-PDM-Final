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
void Graph::insertEdge(const int & vex1, const int & vex2, int cost){
    Edge * new_edge1 = new Edge(vex1, vex2, cost);
    Edge * new_edge2 = new Edge(vex1, vex2, cost);
    numEdge++;

    //add created edge to edge list stored in vertex
    if(VexList[vex1].FirstAdjacentEdge==nullptr)VexList[vex1].FirstAdjacentEdge = new_edge1;
    else{
        Edge* ptr = VexList[vex1].FirstAdjacentEdge;
        while(ptr->next!=nullptr)ptr = ptr->next;
        ptr->next = new_edge1;
    }
    if(VexList[vex2].FirstAdjacentEdge==nullptr)VexList[vex2].FirstAdjacentEdge = new_edge2;
    else{
        Edge* ptr = VexList[vex2].FirstAdjacentEdge;
        while(ptr->next!=nullptr)ptr = ptr->next;
        ptr->next = new_edge2;
    }
}

//member function for PRM planner
PRM::PRM() {
  sub_ = nh_.subscribe("/move_base_simple/goal", 5, &PRM::callback, this);
  edge_pub_ = nh_.advertise<visualization_msgs::Marker>("edge_marker", 10);
  node_pub_ = nh_.advertise<visualization_msgs::Marker>("node_markers", 10);

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
void PRM::node_generation() {
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
        Vertice v(p.x,p.y,p.z);
        graph_.insertVex(v);
        points_.points.push_back(p);
    }
    node_pub_.publish(points_);
}

void PRM::edge_generation() {
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = "world";
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "planner_example";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    line_list.id = 2;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.scale.x = 0.1;
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;

    for(int i=0;i<graph_.get_numVex();i++){
        for(int j=i+1;j<graph_.get_numVex();j++){
            graph_.insertEdge(i,j,1);
            geometry_msgs::Point p1,p2;
            p1.x = graph_.get_vexList()[i].x;
            p1.y = graph_.get_vexList()[i].y;
            p1.z = graph_.get_vexList()[i].z;
            p2.x = graph_.get_vexList()[j].x;
            p2.y = graph_.get_vexList()[j].y;
            p2.z = graph_.get_vexList()[j].z;
            line_list.points.push_back(p1);
            line_list.points.push_back(p2);
        }
    }

    edge_pub_.publish(line_list);
}

void PRM::callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  this->start.x = 0;
  this->start.y = 0;
  this->start.z = 0;
  this->goal = msg->pose.position;
  node_generation();
  edge_generation();
}

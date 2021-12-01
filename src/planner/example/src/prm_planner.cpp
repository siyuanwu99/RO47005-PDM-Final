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

//-----------------
//General functions
//-----------------
/**
 * @brief calculate distance between two vertexs
 * 
 * @param v1 
 * @param v2 
 * @return double :distance
 */
double distance(const Vertice& v1, const Vertice& v2){
    return sqrt(pow(v1.x-v2.x,2)+pow(v1.y-v2.y,2)+pow(v1.z-v2.z,2));
}

//----------------------------------------
//member function for graph implementation
//----------------------------------------
/**
 * @brief  :insert a new vertex to the graph
 * @author :Moji Shi
 * 
 * @param vex1 
 */
void Graph::insertVex(Vertice vex1){

    VexList.push_back(vex1);
    numVex++;

}

/**
 * @brief insert an edge between vex1 and vex2 with weight of cost
 * @author Moji Shi
 * 
 * @param vex1 
 * @param vex2 
 * @param cost 
 */
void Graph::insertEdge(const int & vex1, const int & vex2){
    double cost = distance(VexList[vex1],VexList[vex2]);
    Edge * new_edge1 = new Edge(vex1, vex2, cost);
    Edge * new_edge2 = new Edge(vex1, vex2, cost);
    numEdge++;
    EdgeList.push_back(Edge(vex1, vex2, cost));

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

/**
 * @brief visualization of nodes
 * @author Moji Shi
 * 
 * @param node_pub_ publisher of node in rviz
 */
void Graph::node_visual(ros::Publisher& node_pub_){

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
    for (int i = 0; i < numVex; i++) {;
        geometry_msgs::Point p;
        p.x = VexList[i].x;
        p.y = VexList[i].y;
        p.z = VexList[i].z;
        points_.points.push_back(p);
    }
    node_pub_.publish(points_);

}

/**
 * @brief visualization of edges
 * @author Moji Shi
 * 
 * @param edge_pub_ publisher of edge in rviz
 */
void Graph::edge_visual(ros::Publisher& edge_pub_, vector<double> color){
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = "world";
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "planner_example";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    line_list.id = 2;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.scale.x = 0.02;
    line_list.color.r = color[0];
    line_list.color.g = color[1];
    line_list.color.b = color[2];
    line_list.color.a = 0.8;
    for(int i=0;i<numEdge;i++){
        Vertice v1 = VexList[EdgeList[i].adjacentVexIndex1];
        Vertice v2 = VexList[EdgeList[i].adjacentVexIndex2];
        geometry_msgs::Point p1,p2;
        p1.x = v1.x;
        p1.y = v1.y;
        p1.z = v1.z;
        p2.x = v2.x;
        p2.y = v2.y;
        p2.z = v2.z;
        line_list.points.push_back(p1);
        line_list.points.push_back(p2);
    }
    ROS_INFO("edge number:%i",numEdge);
    edge_pub_.publish(line_list);
}




//-------------------------------
//member function for PRM planner
//-------------------------------
/**
 * @brief Construct a new PRM::PRM object
 * @author Moji Shi
 */
PRM::PRM() {
    sub_ = nh_.subscribe("/move_base_simple/goal", 5, &PRM::callback, this);
    edge_pub_ = nh_.advertise<visualization_msgs::Marker>("edge_marker", 10);
    node_pub_ = nh_.advertise<visualization_msgs::Marker>("node_markers", 10);
    get_map_param();
    //generate initial random graph
    node_generation();
    edge_generation();
}

/**
 * @brief get map parameters from parameter server(size, number of samples)
 * @author Moji Shi
 */
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

/**
 * @brief generate random PRM sample points
 * @author Moji Shi
 */
void PRM::node_generation() {
    // generate random node
    for (int i = 0; i < n_sample; i++) {
        double x,y,z;
        x = ((double)rand() / (RAND_MAX)-0.5) * map_size_x;
        y = ((double)rand() / (RAND_MAX)-0.5) * map_size_y;
        z = ((double)rand() / (RAND_MAX)) * map_size_z;
        ROS_INFO("X:%f  Y:%f  Z:%f", x, y, z);
        Vertice v(x,y,z);
        graph_.insertVex(v);
    }
}

/**
 * @brief generate initial edges
 * @author Moji Shi
 */
void PRM::edge_generation() {
    for(int i=0;i<graph_.get_numVex();i++){
        for(int j=i+1;j<graph_.get_numVex();j++){
            graph_.insertEdge(i,j);
        }
    }
}

/**
 * @brief a_star graph search algorithm
 * @author Moji Shi
 * 
 */
void PRM::a_star(){
    
}

/**
 * @brief call back function of subscriber receiving topic "/move_base_simple/goal"
 * @author Moji Shi
 * 
 * @param msg 
 */
void PRM::callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    this->start.x = 0;
    this->start.y = 0;
    this->start.z = 0;
    this->goal = msg->pose.position;

    //Add goal as a node into the graph
    Vertice end(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
    this->graph_.insertVex(end);
    for(int i=0;i<graph_.get_numVex()-1;i++){
        this->graph_.insertEdge(i, graph_.get_numVex()-1);
    }
    vector<double> color({0,0,1});
    graph_.node_visual(node_pub_);
    graph_.edge_visual(edge_pub_,color);
    a_star();
}

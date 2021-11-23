#include "prm_planner.h"
#include<visualization_msgs/Marker.h>

PRM::PRM(){
    sub_=nh_.subscribe("/move_base_simple/goal",5,&PRM::callback,this);
    pub_=nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    
    get_map_param();
    node_visualization();

}

void PRM::get_map_param(){
    if (nh_.getParam("/random_forest/map/x_size", map_size_x)){
        ROS_INFO("get map x: %f", map_size_x);
    }
    if (nh_.getParam("/random_forest/map/y_size", map_size_y)){
        ROS_INFO("get map y: %f", map_size_y);
    }
    if (nh_.getParam("/random_forest/map/z_size", map_size_z)){
        ROS_INFO("get map z: %f", map_size_z);
    }
}

//Visualization sample nodes
void PRM::node_visualization(){

}

void PRM::callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    this->start.x=0;
    this->start.y=0;
    this->start.z=0;
    this->goal = msg->pose.position;
    PRM::planner_visualization();
}

void PRM::planner_visualization(){
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

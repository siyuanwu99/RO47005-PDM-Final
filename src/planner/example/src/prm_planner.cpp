#include "prm_planner.h"
#include<visualization_msgs/Marker.h>
#include <stdlib.h>

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
    if (nh_.getParam("/prm_planner/number_sample", n_sample)){
        ROS_INFO("get sample number: %i", n_sample);
    }

}

//Visualization sample nodes
void PRM::node_visualization(){
    //generate random node
    visualization_msgs::Marker points;
    points.header.frame_id = "world";
    points.header.stamp = ros::Time::now();
    points.ns = "planner_example";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;
    points.scale.x = 0.2;
    points.scale.y = 0.2;
    points.color.g = 1.0f;
    points.color.a = 1.0;
    for(int i=0;i<n_sample;i++){
        geometry_msgs::Point p;
        p.x = ((double) rand() / (RAND_MAX)-0.5) * map_size_x;
        p.y = ((double) rand() / (RAND_MAX)-0.5) * map_size_y;
        p.z = ((double) rand() / (RAND_MAX)) * map_size_z;
        ROS_INFO("X:%f  Y:%f  Z:%f",p.x,p.y,p.z);
        points.points.push_back(p);
    }
    pub_.publish(points);
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

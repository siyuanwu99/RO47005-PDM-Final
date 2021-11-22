#ifndef _PRM
#define _PRM

#include<ros/ros.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/Point.h>

//PRM based planner
class PRM{
    public:
        PRM();
        void callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void planner_visualization();
    private:
        ros::NodeHandle nh_;
        //subscribe from topic /move_base_simple/goal to get the destination
        ros::Subscriber sub_;
        //publisher the visulization of the algorithm
        ros::Publisher pub_;
        //start and end point 
        geometry_msgs::Point start,goal;
};

#endif
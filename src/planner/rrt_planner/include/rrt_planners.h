#ifndef _RRTS
#define _RRTS

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <map_server/grid_map.h>
#include <vector>
#include <Eigen/Eigen>
#include <numeric>
#include <algorithm>
#include <functional>
#include <Constraint/GridConstraint/GridConstraint.h>
#include <Constraint/PointCloudConstraint/PointCloudConstraint.h>
#include <Planner/InformedRRTStar/InformedRRTStar.h>
#include <Planner/RRT/RRT.h>
#include <Planner/RRTStar/RRTStar.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <chrono>
using std::vector;

/**
 * @brief  :RRT Planner
 * @author :Ranbao Deng
 * 
 */
class RRTS{
    public:
        RRTS(const ros::NodeHandle & nh);
        void callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
        void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void get_map_param();
    private:
        ros::NodeHandle nh_;
        // subscribe from topic /move_base_simple/goal to get the destination
        ros::Subscriber sub_;
        ros::Subscriber odom_sub_;
        ros::Subscriber pnt_cld_sub_;
        // publisher the visulization of the algorithm
        ros::Publisher edge_pub_;
        ros::Publisher node_pub_;
        ros::Publisher path_pub_;
        ros::Publisher path_raw_pub_;
        // Planner
        std::unique_ptr<planner::base::PlannerBase> planner;
        // map size
        double map_size_x,map_size_y,map_size_z;
        int n_sample;
        int mode = 1;
        bool init = false;
        GridMap::Ptr grid_map_ptr_;
        //  current state of quadrator
        Eigen::Vector3d current_pos_;
};

#endif
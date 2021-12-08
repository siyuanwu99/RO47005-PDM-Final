#ifndef _PRM
#define _PRM

#include<ros/ros.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/Point.h>
#include "map_server/grid_map.h"
#include <vector>
#include <Eigen/Eigen>
using std::vector;

/**
 * @brief                    :edge definition in graph
 * @author                   :Moji Shi
 * 
 * @param cost               :weight of the edge
 * @param adjacentVexIndex   :index of vertex which the edge connects
 * @param next               :point to the next edge in adjacent list of edges
 */
struct Edge{
    double cost;
    int adjacentVexIndex1, adjacentVexIndex2;
    struct Edge *next;
    Edge(int b, int c, int a) : adjacentVexIndex1(b), adjacentVexIndex2(c), cost(a), next(nullptr){};
};


/**
 * @brief                   :vertex definition in graph
 * @author                  :Moji Shi
 * 
 * @param xyz               :vertex coordinates in 3D space
 * @param FirstAdjacentEdge :point to the first neighbor edge
 */
struct Vertice{
    double x,y,z;
    Edge * FirstAdjacentEdge;
    Vertice(double a, double b, double c) : x(a), y(b), z(c), FirstAdjacentEdge(nullptr){};
    bool operator==(const Vertice & v){
        return v.x==x && v.y==y && v.z==z;
    }  
};

/**
 * @brief  :class of graph
 * @author :Moji Shi
 * 
 */
class Graph{
    public:
        //insert new node
        void insertVex(Vertice vex1);
        //insert new edge(vex1 vex2 are index in VexList)
        void insertEdge(const int & vex1, const int & vex2);
        void removeEdge(const int & vex1, const int & vex2);
        //visualization 
        void node_visual(ros::Publisher& node_pub_);
        void edge_visual(ros::Publisher& edge_pub_, vector<double> color, double width);
        void clear_graph();
        int get_numVex(){return numVex;};
        int get_numEdge(){return numEdge;};
        vector<Vertice>& get_vexList(){return VexList;};
        vector<Edge>& get_EdgeList(){return EdgeList;};
    private:
        vector<Edge> EdgeList;
        vector<Vertice> VexList;
        int numVex = 0, numEdge = 0;

};

/**
 * @brief  :PRM Planner
 * @author :Moji Shi
 * 
 */
class PRM{
    public:
        PRM(const ros::NodeHandle & nh);
        void callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
        void planner_visualization();
        void node_generation();
        void edge_generation();
        void node_visual();
        void edge_visual();
        void get_map_param();
        void rate_publisher();
        void a_star();
        void clear();
        bool collision_check(const Vertice&p);
        bool collision_check(const Vertice&p1, const Vertice&p2);
        int start_idx, goal_idx;
    private:
        ros::NodeHandle nh_;
        //subscribe from topic /move_base_simple/goal to get the destination
        ros::Subscriber sub_;
        ros::Subscriber pnt_cld_sub_;
        //publisher the visulization of the algorithm
        ros::Publisher edge_pub_;
        ros::Publisher node_pub_;
        ros::Publisher path_pub_;
        ros::Publisher path_raw_pub_;
        //start and end point 
        geometry_msgs::Point start,goal;
        //map size
        double map_size_x,map_size_y,map_size_z;
        //sample number
        int n_sample;
        //graph
        Graph graph_;
        GridMap::Ptr grid_map_ptr_;
        bool is_graph_generated;
};

#endif
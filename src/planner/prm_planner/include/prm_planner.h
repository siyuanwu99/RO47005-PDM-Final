#ifndef _PRM
#define _PRM

#include<ros/ros.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include "map_server/grid_map.h"
#include <vector>
#include <Eigen/Eigen>
#include <numeric>
#include <algorithm>
#include <functional>
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

typedef Vertice* VerticePtr;  // will be used in Flight Corridor

/**
 * @brief                   :k-d tree node.
 * @author                  :Ranbao Deng
 * 
 * @param idx               :index to the original point
 * @param next              :pointers to the child nodes
 * @param axis              :dimension's axis
 */
struct Node
{
    int idx; 
    Node* next[2];
    int axis;

    Node() : idx(-1), axis(-1) { next[0] = next[1] = nullptr; }
};

/**
 * @brief                   :k-d tree new node.
 * @author                  :Ranbao Deng
 * 
 */
struct Node* newNode(int index, int depth)
{
    struct Node* temp = new Node;

    temp->idx = index;
    temp->axis = depth % 3;
    temp->next[0] = temp->next[1] = NULL;
    return temp;
}

/**
 * @brief                   :Class of KDTree
 * @author                  :gishi523, Ranbao Deng
 * 
 */
class KDTree{
    public:
        KDTree() : root_(nullptr) {};
        KDTree(const vector<Vertice>& points) : root_(nullptr) { build(points); }
        ~KDTree() { clear(); }

        void build(const vector<Vertice>& points);
        void clear();

        int nnSearch(const Vertice& query, double* minDist);
        vector<int> knnSearch(const Vertice& query, int k);
        vector<int> radiusSearch(const Vertice& query, double radius);
        vector<Vertice>& get_vexList(){ return points_; };
        Node* rootNode(){ return root_; };
        Node* insertNode(Vertice point);        

    private:
        /** @brief Bounded priority queue.
        */
        template <class T, class Compare = std::less<T>>
        class BoundedPriorityQueue
        {
        public:

            BoundedPriorityQueue() = delete;
            BoundedPriorityQueue(size_t bound) : bound_(bound) { elements_.reserve(bound + 1); };

            void push(const T& val)
            {
                auto it = std::find_if(std::begin(elements_), std::end(elements_),
                    [&](const T& element){ return Compare()(val, element); });
                elements_.insert(it, val);

                if (elements_.size() > bound_)
                    elements_.resize(bound_);
            }

            const T& back() const { return elements_.back(); };
            const T& operator[](size_t index) const { return elements_[index]; }
            size_t size() const { return elements_.size(); }

        private:
            size_t bound_;
            std::vector<T> elements_;
        };

        /** @brief Priority queue of <distance, index> pair.
        */
        using KnnQueue = BoundedPriorityQueue<std::pair<double, int>>;   

        Node* buildRecursive(int* indices, int npoints, int depth);
        Node* insertNodeRec(Node *node, int index, int depth);
        void clearRecursive(Node* node);
        void nnSearchRecursive(const Vertice& query, const Node* node, int *guess, double *minDist);
        void knnSearchRecursive(const Vertice& query, const Node* node, KnnQueue& queue, int k);
        void radiusSearchRecursive(const Vertice& query, const Node* node, std::vector<int>& indices, double radius);

        Node* root_;                  // root node of KDTree
        std::vector<Vertice> points_; // points
};

/**
 * @brief  :class of graph
 * @author :Moji Shi
 * 
 */
class Graph{
    public:
        // insert new node
        void insertVex(Vertice vex1);
        // insert new edge(vex1 vex2 are index in VexList)
        void insertEdge(const int & vex1, const int & vex2);
        void removeEdge(const int & vex1, const int & vex2);
        // visualization 
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
        void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
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
        vector<int> start_knn_idxs, goal_knn_idxs;
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
        // start and end point 
        geometry_msgs::Point start,goal;
        // map size
        double map_size_x,map_size_y,map_size_z;
        // sample number
        int n_sample;
        int k;
        // graph
        Graph graph_;
        GridMap::Ptr grid_map_ptr_;
        bool is_graph_generated;
        // tree
        KDTree tree_;
        //  current state of quadrator
        Eigen::Vector3d current_pos_;
};

#endif
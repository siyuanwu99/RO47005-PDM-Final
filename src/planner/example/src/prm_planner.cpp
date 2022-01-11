/**
 * @file prm_planner.cpp
 * @author Moji Shi
 * @brief 
 * @version 1.0
 * @date 2021-12-02
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "prm_planner.h"

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

/**
 * @brief transform axis to xyz
 * 
 * @param p 
 * @param axis 
 * @return double :position at xyz
 */
static double axisToXYZ(const Vertice& p, const int axis)
{
    double xyz;
    if (axis == 0) 
        xyz = p.x;
    else if (axis == 1)
        xyz = p.y;
    else if (axis == 2)
        xyz = p.z;
    return xyz;
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

void Graph::clear_graph(){
    EdgeList.clear();
    VexList.clear();
    numVex = 0;
    numEdge = 0;
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
 * @brief remove edge
 * @author Moji Shi
 * 
 */
void Graph::removeEdge(const int & vex1, const int & vex2){
    if(VexList[vex1].FirstAdjacentEdge->adjacentVexIndex1 == vex2 || VexList[vex1].FirstAdjacentEdge->adjacentVexIndex2 == vex2){
        Edge *temp = VexList[vex1].FirstAdjacentEdge;
        delete VexList[vex1].FirstAdjacentEdge;
        VexList[vex1].FirstAdjacentEdge = temp->next;
    }
    else{
        Edge* ptr = VexList[vex1].FirstAdjacentEdge;
        while(ptr->next->adjacentVexIndex1 != vex2 && ptr->next->adjacentVexIndex2 != vex2 && ptr->next != nullptr)ptr = ptr->next;
        Edge *temp = ptr->next;
        delete ptr->next;
        ptr->next = temp->next;
    }

    if(VexList[vex2].FirstAdjacentEdge->adjacentVexIndex1 == vex1 || VexList[vex2].FirstAdjacentEdge->adjacentVexIndex2 == vex1){
        Edge *temp = VexList[vex2].FirstAdjacentEdge;
        delete VexList[vex2].FirstAdjacentEdge;
        VexList[vex2].FirstAdjacentEdge = temp->next;
    }
    else{
        Edge* ptr = VexList[vex2].FirstAdjacentEdge;
        while(ptr->next->adjacentVexIndex1 != vex1 && ptr->next->adjacentVexIndex2 != vex1 && ptr->next != nullptr)ptr = ptr->next;
        Edge *temp = ptr->next;
        delete ptr->next;
        ptr->next = temp->next;
    }

    for(auto itr=EdgeList.begin(); itr!=EdgeList.end(); itr++){
        if((itr->adjacentVexIndex1 == vex1 && itr->adjacentVexIndex2 == vex2) || (itr->adjacentVexIndex1 == vex2 && itr->adjacentVexIndex2 == vex1)){
            EdgeList.erase(itr);
            break;
        }
    }

    numEdge--;

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
    points_.ns = "planner";
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
void Graph::edge_visual(ros::Publisher& edge_pub_, vector<double> color, double width){
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = "world";
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "planner";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    line_list.id = 2;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.scale.x = width;
    line_list.color.r = color[0];
    line_list.color.g = color[1];
    line_list.color.b = color[2];
    line_list.color.a = 0.8;
    double cost=0;
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
        cost += distance(v1, v2);
    }
    ROS_INFO("edges cost : %f", cost);
    edge_pub_.publish(line_list);
}



//----------------------------------------
//member function for KDTree implementation
//----------------------------------------
/**
 * @brief        :Build k-d tree
 * @author       :Ranbao Deng
 * 
 * @param points :vector of vertices
 */
void KDTree::build(const vector<Vertice>& points){
    clear();    

    points_ = points;

    vector<int> indices(points.size());
    std::iota(std::begin(indices), std::end(indices), 0);

    root_ = buildRecursive(indices.data(), (int)points.size(), 0);
}

/**
 * @brief        :Clear k-d tree
 * @author       :Ranbao Deng
 * 
 */
void KDTree::clear(){
    clearRecursive(root_);
    root_ = nullptr;
    points_.clear();
}

/**
 * @brief        :Searches the nearest neighbor
 * @author       :Ranbao Deng
 * 
 * @param query  :query vertice
 * @param minDist:minimum distance
 */
int KDTree::nnSearch(const Vertice& query, double* minDist = nullptr)
{
    int guess;
    double _minDist = std::numeric_limits<double>::max();

    nnSearchRecursive(query, root_, &guess, &_minDist);

    if (minDist)
        *minDist = _minDist;
    
    return guess;
}

/**
 * @brief        :Searches the k-nearest neighbor
 * @author       :Ranbao Deng
 * 
 * @param query  :query vertice
 * @param k      :k
 */
vector<int> KDTree::knnSearch(const Vertice& query, int k)
{
    KnnQueue queue(k);
    knnSearchRecursive(query, root_, queue, k);
    
    vector<int> indices(queue.size());
    for (size_t i = 0; i < queue.size(); i++)
        indices[i] = queue[i].second;
    
    return indices;
}

/**
 * @brief        :Searches neighbors within radius
 * @author       :Ranbao Deng
 * 
 * @param query  :query vertice
 * @param radius :radius
 */
vector<int> KDTree::radiusSearch(const Vertice& query, double radius)
{
    vector<int> indices;
    radiusSearchRecursive(query, root_, indices, radius);
    return indices;
}

/**
 * @brief        :Insert node
 * @author       :Ranbao Deng
 * 
 * @param point  :vertice to insert
 */
Node* KDTree::insertNode(Vertice point)
{
    points_.push_back(point);
    return insertNodeRec(root_, points_.size()-1, 0);
}

/**
 * @brief        :Builds k-d tree recursively.
 * @author       :Ranbao Deng
 * 
 * @param indices:pointer to the first vertice
 * @param npoints:points length
 * @param depth  :depth or axis of the current node
 */
Node* KDTree::buildRecursive(int* indices, int npoints, int depth)
{
    if (npoints <= 0)
        return nullptr;

    const int axis = depth % 3;
    const int mid = (npoints - 1) / 2;

    std::nth_element(indices, indices + mid, indices + npoints, [&](int lhs, int rhs)
    {
        if (axis == 0) 
            return points_[lhs].x < points_[rhs].x;
        else if (axis == 1)
            return points_[lhs].y < points_[rhs].y;
        else if (axis == 2)
            return points_[lhs].z < points_[rhs].z;
    });

    Node* node = new Node();
    node->idx = indices[mid];
    node->axis = axis;

    node->next[0] = buildRecursive(indices, mid, depth + 1);
    node->next[1] = buildRecursive(indices + mid + 1, npoints - mid - 1, depth + 1);

    return node;
}

/**
 * @brief        :Insert node
 * @author       :Ranbao Deng
 * 
 * @param node   :pointer to the current node
 * @param index  :index of the point to insert
 * @param depth  :depth or axis of the current node
 */
Node* KDTree::insertNodeRec(Node *node, int index, int depth)
{            
    // Tree is empty?
    if (node == nullptr)
        return newNode(index, depth);

    // Calculate current dimension (cd) of comparison
    const int axis = depth % 3;

    // Compare the new point with root on current dimension 'cd'
    // and decide the left or right subtree
    if ( axisToXYZ(points_[index], axis) < (axisToXYZ(points_[node->idx], axis)))
        node->next[0] = insertNodeRec(node->next[0], index, depth + 1);
    else
        node->next[1] = insertNodeRec(node->next[1], index, depth + 1);
    return node;
}

/**
 * @brief        :clear node recursively
 * @author       :Ranbao Deng
 * 
 * @param node   :pointer to the current node
 */
void KDTree::clearRecursive(Node* node)
{
    if (node == nullptr)
        return;

    if (node->next[0])
        clearRecursive(node->next[0]);

    if (node->next[1])
        clearRecursive(node->next[1]);

    delete node;
}

/**
 * @brief        :Searches the nearest neighbor recursively
 * @author       :Ranbao Deng
 * 
 * @param query  :query vertice
 * @param node   :pointer to the current node
 * @param guess  :nearest guess
 * @param minDist:minimum distance
 */
void KDTree::nnSearchRecursive(const Vertice& query, const Node* node, int *guess, double *minDist)
{
    if (node == nullptr)
        return;

    const Vertice& train = points_[node->idx];

    const double dist = distance(query, train);
    if (dist < *minDist)
    {
        *minDist = dist;
        *guess = node->idx;
    }

    const int axis = node->axis;
    const int dir = axisToXYZ(query, axis) < axisToXYZ(train, axis) ? 0 : 1;
    nnSearchRecursive(query, node->next[dir], guess, minDist);

    const double diff = fabs(axisToXYZ(query, axis) - axisToXYZ(train, axis));
    if (diff < *minDist)
        nnSearchRecursive(query, node->next[!dir], guess, minDist);
}

/**
 * @brief        :Searches k-nearest neighbors recursively
 * @author       :Ranbao Deng
 * 
 * @param query  :query vertice
 * @param node   :pointer to the current node
 * @param queue  :knn queue
 * @param k      :k
 */
void KDTree::knnSearchRecursive(const Vertice& query, const Node* node, KnnQueue& queue, int k)
{
    if (node == nullptr)
        return;

    const Vertice& train = points_[node->idx];

    const double dist = distance(query, train);
    queue.push(std::make_pair(dist, node->idx));

    const int axis = node->axis;
    const int dir = axisToXYZ(query, axis) < axisToXYZ(train, axis) ? 0 : 1;
    knnSearchRecursive(query, node->next[dir], queue, k);

    const double diff = fabs(axisToXYZ(query, axis) - axisToXYZ(train, axis));
    if ((int)queue.size() < k || diff < queue.back().first)
        knnSearchRecursive(query, node->next[!dir], queue, k);
}

/**
 * @brief        :Searches neighbors within radius
 * @author       :Ranbao Deng
 * 
 * @param query  :query vertice
 * @param node   :pointer to the current node
 * @param indices:indices
 * @param radius :radius
 */
void KDTree::radiusSearchRecursive(const Vertice& query, const Node* node, std::vector<int>& indices, double radius)
{
    if (node == nullptr)
        return;

    const Vertice& train = points_[node->idx];

    const double dist = distance(query, train);
    if (dist < radius)
        indices.push_back(node->idx);

    const int axis = node->axis;
    const int dir = axisToXYZ(query, axis) < axisToXYZ(train, axis) ? 0 : 1;
    radiusSearchRecursive(query, node->next[dir], indices, radius);

    const double diff = fabs(axisToXYZ(query, axis) - axisToXYZ(train, axis));
    if (diff < radius)
        radiusSearchRecursive(query, node->next[!dir], indices, radius);
}



//-------------------------------
//member function for PRM planner
//-------------------------------
/**
 * @brief Construct a new PRM::PRM object
 * @author Moji Shi
 */
PRM::PRM(const ros::NodeHandle & nh) {

    nh_ = nh;

    grid_map_ptr_.reset(new GridMap);
    grid_map_ptr_->initGridMap(nh_);
    pnt_cld_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
      "/mock_map", 10, &PRM::pointCloudCallback, this);
    sub_ = nh_.subscribe("/move_base_simple/goal", 5, &PRM::callback, this);
    odom_sub_ = nh_.subscribe("odometry", 5, &PRM::OdomCallback, this);
    edge_pub_ = nh_.advertise<visualization_msgs::Marker>("edge_marker", 10);
    path_pub_ = nh_.advertise<visualization_msgs::Marker>("path_marker", 10);
    node_pub_ = nh_.advertise<visualization_msgs::Marker>("node_markers", 10);
    path_raw_pub_ = nh_.advertise<geometry_msgs::PoseArray>("raw_path", 10);
    get_map_param();
    init = false;
    k = 250;
}

void PRM::clear(){
    graph_.clear_graph();
}

/**
 * @brief get map parameters from parameter server(size, number of samples)
 * @author Moji Shi
 */
void PRM::get_map_param() {
  if (nh_.getParam("map_x_size", map_size_x)) {
    ROS_INFO("prm get map x: %f", map_size_x);
  }
  if (nh_.getParam("map_y_size", map_size_y)) {
    ROS_INFO("prm get map y: %f", map_size_y);
  }
  if (nh_.getParam("map_z_size", map_size_z)) {
    ROS_INFO("prm get map z: %f", map_size_z);
  }
  if (nh_.getParam("number_sample", n_sample)) {
    ROS_INFO("prm get sample number: %i", n_sample);
  }
}

/**
 * @brief generate random PRM sample points
 * @author Moji Shi
 */
void PRM::node_generation() {
    // generate random node
    Vertice v1(-10,-10,1);
    graph_.insertVex(v1);
    Vertice v0(0,0,0);
    graph_.insertVex(v0);
    for (int i = 0; i < n_sample; i++) {
        double x,y,z;
        x = ((double)rand() / (RAND_MAX)-0.5) * map_size_x;
        y = ((double)rand() / (RAND_MAX)-0.5) * map_size_y;
        z = ((double)rand() / (RAND_MAX)+0.1) * map_size_z*0.8;
        Vertice v(x,y,z);
        if(collision_check(v))graph_.insertVex(v);
    }
    ROS_INFO("Nodes generated with %d samples",n_sample);
    tree_.build(graph_.get_vexList()); //KDTree    
}

/**
 * @brief generate initial edges
 * @author Moji Shi, Ranbao Deng
 */
void PRM::edge_generation() {
    vector<int> knn_idxs;

    if (k!=0) {        
        ROS_INFO("Edges generated with %d-nearest neighbours",k);
    } else {
        ROS_INFO("Edges generated with full connection");
    }
    
    for(int i=0;i<graph_.get_numVex();i++){
        if (k!=0) {
            knn_idxs = tree_.knnSearch(graph_.get_vexList()[i], k); //KDTree K can not be lower than 230, otherwise a* will freeze easily.
            // insert edge with knn
            for(int j=1;j<knn_idxs.size();j++){
                // if(knn_idxs[j]>i){
                    if(collision_check(graph_.get_vexList()[i], graph_.get_vexList()[knn_idxs[j]])) {
                        graph_.insertEdge(i,knn_idxs[j]);
                    }
                // }
            }
        } else {
            for(int j=i+1;j<graph_.get_numVex();j++){
                if(collision_check(graph_.get_vexList()[i], graph_.get_vexList()[j])) {
                    graph_.insertEdge(i,j);
                }
            }
        }        
    }
}

/**
 * @brief a_star graph search algorithm
 * @author Moji Shi
 * 
 */
void PRM::a_star(){
    ROS_INFO("A* operating...");

    vector<double> visited(graph_.get_numVex(),-1);
    vector<int> pre(graph_.get_numVex(),-1);
    std::priority_queue<std::tuple<double, int, int>, vector<std::tuple<double, int, int>>, std::greater<std::tuple<double, int, int>>> q;
    q.push(std::make_tuple(0+distance(graph_.get_vexList()[start_idx], graph_.get_vexList()[goal_idx]), start_idx, start_idx));
    int cur = start_idx;

    while(!q.empty() && cur!=goal_idx){
        cur = std::get<1>(q.top());
        pre[cur] = std::get<2>(q.top());
        visited[cur] = visited[pre[cur]] + distance(graph_.get_vexList()[pre[cur]], graph_.get_vexList()[cur]);
        q.pop();
        //insert all neighbour node to queue
        if(graph_.get_vexList()[cur].FirstAdjacentEdge!=nullptr){

            Edge* ptr = graph_.get_vexList()[cur].FirstAdjacentEdge;
            if(ptr->adjacentVexIndex1==cur && visited[ptr->adjacentVexIndex2]==-1){
                q.push(std::make_tuple(visited[cur] + ptr->cost + distance(graph_.get_vexList()[ptr->adjacentVexIndex2], graph_.get_vexList()[goal_idx]), ptr->adjacentVexIndex2, cur));
            }
            else if(ptr->adjacentVexIndex2==cur && visited[ptr->adjacentVexIndex1]==-1){
                q.push(std::make_tuple(visited[cur] + ptr->cost + distance(graph_.get_vexList()[ptr->adjacentVexIndex1], graph_.get_vexList()[goal_idx]), ptr->adjacentVexIndex1, cur));
            }
            
            while(ptr->next!=nullptr){
                ptr = ptr->next;
                if(ptr->adjacentVexIndex1==cur && visited[ptr->adjacentVexIndex2]==-1){
                q.push(std::make_tuple(visited[cur] + ptr->cost + distance(graph_.get_vexList()[ptr->adjacentVexIndex2], graph_.get_vexList()[goal_idx]), ptr->adjacentVexIndex2, cur));
                }
                else if(ptr->adjacentVexIndex2==cur && visited[ptr->adjacentVexIndex1]==-1){
                    q.push(std::make_tuple(visited[cur] + ptr->cost + distance(graph_.get_vexList()[ptr->adjacentVexIndex1], graph_.get_vexList()[goal_idx]), ptr->adjacentVexIndex1, cur));
                }
            } 
        }
    }

    Graph G;
    G.insertVex(graph_.get_vexList()[goal_idx]);
    int cur_idx = goal_idx;
    //check if the path is found
    if(pre[cur_idx] == -1){
        ROS_INFO("A* no path found, cost : 0");
    }
    else{
        while(cur_idx!=start_idx){
            int pre_idx = pre[cur_idx];
            G.insertVex(graph_.get_vexList()[pre_idx]);
            G.insertEdge(G.get_numVex()-2, G.get_numVex()-1);
            cur_idx = pre_idx;
        }
        vector<double> color = {0.1,0.1,0.1};
        G.edge_visual(path_pub_, color, 0.15);

        //pass path to traj optimization
        geometry_msgs::PoseArray raw_path;
        raw_path.header.frame_id = "world";
        raw_path.header.stamp = ros::Time::now();


        geometry_msgs::Pose start_point;
        start_point.position.x = G.get_vexList().rbegin()->x;
        start_point.position.y = G.get_vexList().rbegin()->y;
        start_point.position.z = G.get_vexList().rbegin()->z;
        // raw_path.poses.push_back(start_point);
        // raw_path.poses.push_back(start_point);

        // double cost=0;
        // geometry_msgs::Pose pre_point;
        for(auto itr=G.get_vexList().rbegin(); itr!=G.get_vexList().rend(); itr++){            
            geometry_msgs::Pose point;
            point.position.x = itr->x;
            point.position.y = itr->y;
            point.position.z = itr->z;
            raw_path.poses.push_back(point);
            // cost += sqrt(pow(point.position.x-pre_point.position.x,2)+pow(point.position.y-pre_point.position.y,2)+pow(point.position.z-pre_point.position.z,2));
            // pre_point = point;
        }
        // ROS_INFO("cost : %f", cost);

        geometry_msgs::Pose end_point;
        end_point.position.x = (G.get_vexList().rend()-1)->x;
        end_point.position.y = (G.get_vexList().rend()-1)->y;
        end_point.position.z = (G.get_vexList().rend()-1)->z;
        // raw_path.poses.push_back(end_point);
        // raw_path.poses.push_back(end_point);

        path_raw_pub_.publish(raw_path);
        ROS_INFO("A* solution found");
    }    
}

/**
 * @brief collision check for node
 * @author Siyuan Wu
 * 
 * @param p vertice
 * @return true if no collision
 * @return false 
 */
bool PRM::collision_check(const Vertice& p){
    Eigen::Vector3f pt;
    pt(0) = static_cast<float>(p.x);
    pt(1) = static_cast<float>(p.y);
    pt(2) = static_cast<float>(p.z);
    return !grid_map_ptr_->isPointCollision(pt);
}
/**
 * @brief collision check for edge
 * @author Siyuan Wu
 * 
 * @param e edge
 * @return true 
 * @return false 
 */
bool PRM::collision_check(const Vertice& p1, const Vertice& p2){
    Eigen::Vector3f p1e, p2e;
    p1e(0) = static_cast<float>(p1.x);
    p1e(1) = static_cast<float>(p1.y);
    p1e(2) = static_cast<float>(p1.z);
    p2e(0) = static_cast<float>(p2.x);
    p2e(1) = static_cast<float>(p2.y);
    p2e(2) = static_cast<float>(p2.z);
    return !grid_map_ptr_->isStraightLineCollision(p1e, p2e);
}


/**
 * @brief call back function of subscriber receiving topic "/move_base_simple/goal"
 * @author Moji Shi, Ranbao Deng
 * 
 * @param msg 
 */
void PRM::callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    // if (!is_graph_generated){
    // // clear();
    // //generate initial random graph
    // node_generation();
    // edge_generation();
    // is_graph_generated = true;
    // }

    // Add current position as start position
    // Vertice start(current_pos_(0), current_pos_(1), current_pos_(2));
    Vertice start(-10, -10, 0);
    // Prevent same vertice if not moving
    // if ((current_pos_(0) + current_pos_(1) + current_pos_(2) < 0.001) && (current_pos_(0) + current_pos_(1) + current_pos_(2) > -0.001)){
    //     Vertice start(current_pos_(0)+((double)rand() / (RAND_MAX)-0.5)/10, current_pos_(1)+((double)rand() / (RAND_MAX)-0.5)/10, current_pos_(2)+((double)rand() / (RAND_MAX)-0.5)/10);
    // };    
    // ROS_INFO("position received: %f, %f, %f",current_pos_(0),current_pos_(1),current_pos_(2));
    // ROS_INFO("position received");

    //Add goal as a node into the graph
    Vertice end(msg->pose.position.x+((double)rand() / (RAND_MAX)-0.5)/100,msg->pose.position.y,msg->pose.position.z);
    ROS_INFO("position received: %f, %f, %f",msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
    // this->graph_.insertVex(start);
    // this->tree_.insertNode(start); //KDTree

    start_idx = 0;
    if(collision_check(end)){

        for(int i=1;i<100;i++){
            this->graph_.insertVex(end);
            // this->tree_.insertNode(end); //KDTree

            goal_idx = graph_.get_numVex()-1;
            //If target is valid, run graph search
            // n_sample = 500;
            
            auto start_time = std::chrono::system_clock::now();
            tree_.build(graph_.get_vexList()); //KDTree

            // start_knn_idxs = tree_.knnSearch(start, 10); //KDTree
            // ROS_INFO("start_knn: %d",(int)start_knn_idxs.size()); //KDTree

            // for(int i=0;i<graph_.get_numVex()-1;i++){
            //     if(collision_check(graph_.get_vexList()[i], graph_.get_vexList()[graph_.get_numVex()-1])){
            //         this->graph_.insertEdge(i, start_idx);
            //     }
            // }

            // for(int i=0;i<start_knn_idxs.size();i++){
            //     if(start_knn_idxs[i]<start_idx){
            //         if(collision_check(graph_.get_vexList()[start_knn_idxs[i]], graph_.get_vexList()[start_idx])){
            //             this->graph_.insertEdge(start_knn_idxs[i], start_idx);
            //         }
            //     }
            // }        

            //KDTree
            // ROS_INFO("goal_knn_idxs: %d",(int)goal_knn_idxs.size()); //KDTree

            // for(int i=0;i<graph_.get_numVex()-1;i++){
            //     if(collision_check(graph_.get_vexList()[i], graph_.get_vexList()[graph_.get_numVex()-1])){
            //         this->graph_.insertEdge(i, goal_idx);
            //     }
            // }
            if (k!=0){
                goal_knn_idxs = tree_.knnSearch(end, k);
                // ROS_INFO("goal_knn_idxs: %d",(int)goal_knn_idxs.size());
                for(int i=1;i<goal_knn_idxs.size();i++){                    
                    if(collision_check(graph_.get_vexList()[goal_knn_idxs[i]], graph_.get_vexList()[goal_idx])){
                        // ROS_INFO("goal_knn_idxs: %d with %d",goal_knn_idxs[i],goal_idx);
                        this->graph_.insertEdge(goal_knn_idxs[i], goal_idx);
                    }
                }
            }else{
                for(int i=0;i<graph_.get_numVex()-1;i++){
                    if(collision_check(graph_.get_vexList()[i], graph_.get_vexList()[graph_.get_numVex()-1])){
                        this->graph_.insertEdge(i, goal_idx);
                    }
                }            
            }           

            

            //Visualize new graph
            vector<double> color({0,0,1});
            graph_.node_visual(node_pub_);
            // graph_.edge_visual(edge_pub_,color, 0.02);
            // grid_map_ptr_->publish();
   
            a_star();
            auto end_time = std::chrono::system_clock::now();
            ROS_INFO_STREAM(k<<"-prm elapsed time : " << std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count() / 1000.0 << "ms");
            // graph_.clear_graph();
            // auto init1 = std::chrono::system_clock::now();
            // node_generation();
            // edge_generation();
            // auto init2 = std::chrono::system_clock::now();
            // ROS_INFO_STREAM("init time : " << std::chrono::duration_cast<std::chrono::microseconds>(init2 - init1).count() / 1000.0 << "ms");
            // this->graph_.insertVex(end);
            // // this->tree_.insertNode(end); //KDTree

            // goal_idx = graph_.get_numVex()-1;
        }        
        ROS_INFO("Loop complete.");
    }
    else{
        ROS_INFO("invalid target!");
    }
    
}

void PRM::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    if (!init){
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);
        int cloud_size = cloud.points.size();
        ROS_INFO("Received point cloud : %d",cloud_size);
        grid_map_ptr_->pointCloudCallback(msg);
        auto init1 = std::chrono::system_clock::now();
        node_generation();
        edge_generation();
        auto init2 = std::chrono::system_clock::now();
        ROS_INFO_STREAM("Initialization time : " << std::chrono::duration_cast<std::chrono::microseconds>(init2 - init1).count() / 1000.0 << "ms");
        ROS_INFO("PRM SAMPLING FINIHED");
        init = true;
    }
}

void PRM::OdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_pos_(0) = msg->pose.pose.position.x;
    current_pos_(1) = msg->pose.pose.position.y;
    current_pos_(2) = msg->pose.pose.position.z;
}

void PRM::rate_publisher() {
    grid_map_ptr_->publish();
}
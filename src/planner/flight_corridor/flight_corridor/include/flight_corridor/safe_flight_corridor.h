/**
 * @file safe_flight_corridor.h
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2021-12-18
 *
 * Original created by Han-Sin in ZJU-FAST-Lab
 * Modified by Siyuan Wu to fit our project
 *
 */
#ifndef _SFC_H
#define _SFC_H
#include <nav_msgs/Path.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include "map_server/grid_map.h"

#include <Eigen/Eigen>
#include <iostream>
#include <utility>

struct Edge {
  double cost;
  int adjacentVexIndex1, adjacentVexIndex2;
  struct Edge *next;
  Edge(int b, int c, int a)
      : adjacentVexIndex1(b), adjacentVexIndex2(c), cost(a), next(nullptr) {}
};

struct Node {
  double x, y, z;
  Edge *FirstAdjacentEdge;
  Node(double a, double b, double c)
      : x(a), y(b), z(c), FirstAdjacentEdge(nullptr) {}
  bool operator==(const Node &v) { return v.x == x && v.y == y && v.z == z; }
};

typedef Node *NodePtr;  // will be used in Flight Corridor
class FlightCube {
 public:
  NodePtr start;
  NodePtr end;
  double x_pos;
  double x_neg;
  double y_pos;
  double y_neg;
  double z_pos;
  double z_neg;
  int x_pos_int;
  int x_neg_int;
  int y_pos_int;
  int y_neg_int;
  int z_pos_int;
  int z_neg_int;
  int borders_int[6];  // 0 for xl,1 for xu,2 for yl,3 for yu,4 for zl,5 for z
  double borders[6];

  FlightCube(NodePtr s_n, NodePtr e_n);

  void DisplayCube();
};

class FlightCorridor {
 public:
  std::vector<FlightCube> cubes_;
  // int max_expand_size;

  bool isCubeSafe(FlightCube cube,
                  std::vector<Eigen::Vector3i> &collision_grids,
                  int last_node_order, int check_order, int flag = 0);
  bool isCubeSafe(FlightCube cube, int flag = 0);

  FlightCube expandCube(FlightCube &cube);

  void updateAttributes(FlightCube &cube);

  void divideCorridor(double min_length);

  void divide_one_corridor();

  Eigen::MatrixXd corridor2mat();

  int generate(std::vector<Eigen::Vector3d> &gridpath);

  void set_map(GridMap::Ptr env) { gridmap_ = env; }

 private:
  GridMap::Ptr gridmap_;
};

#endif  // _SFC_H

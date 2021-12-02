/**
 * @file test_grid_map_node.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2021-12-02
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "map_server/grid_map.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_map_server_node");

  ros::NodeHandle nh("~");
  ros::Rate r(1);
  GridMap grid_map;
  grid_map.initGridMap(nh);

  while (ros::ok()) {
    grid_map.publish();
    ros::spinOnce();
    r.sleep();
  }
}

/**
 * TO debug
 * rosrun map_server test_grid_map_node /grid_map/x_size:=/random_forest/map/x_size /grid_map/y_size:=/random_forest/map/y_size /grid_map/z_size:=/random_forest/map/z_size ~cloud_in:=/map_generator/global_cloud
 */
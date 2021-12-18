/**
 * @file safe_flight_corridor.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 *
 * @version 1.0
 * @date 2021-12-18
 *
 * Original created by Han-Sin in ZJU-FAST-Lab
 * Modified by Siyuan Wu to fit our project
 *
 */

#include <sfc_generation/sfc.h>

/*****************************************************/
/******************** Flight Cube ********************/
/*****************************************************/

// TODO: int or double? what does index means in Node class
FlightCube::FlightCube(NodePtr s_n, NodePtr e_n) {
  start_node = s_n;
  end_node = e_n;
  if (end_node.x > start_node.x) {  // init x
    x_pos_int = end_node.x - start_node.x;
    x_neg_int = 0;
  } else {
    x_neg_int = start_node.x - end_node.x;
    x_pos_int = 0;
  }

  if (end_node.y > start_node.y) {  // init y
    y_pos_int = end_node.y - start_node.y;
    y_neg_int = 0;
  } else {
    y_neg_int = start_node.y - end_node.y;
    y_pos_int = 0;
  }

  if (end_node.z > start_node.z) {  // init z
    z_pos_int = end_node.z - start_node.z;
    z_neg_int = 0;
  } else {
    z_neg_int = start_node.z - end_node.z;
    z_pos_int = 0;
  }
}

void DisplayCube() {
  // ROS_INFO("start_node_x_int=%d   y_int=%d   z_int=%d     x_pos_int=%d
  // y_pos_int=%d  z_pos_int=%d  x_neg_int=%d  y_neg_int=%d  z_neg_int=%d",
  // start_node.x,start_node.y,start_node.z,x_pos_int,y_pos_int,z_pos_int,x_neg_int,y_neg_int,z_neg_int);

  ROS_INFO(
      "start_node_x=%f   y=%f  z=%f     x_pos=%f  y_pos=%f  z_pos=%f  "
      "x_neg=%f  y_neg=%f  z_neg=%f",
      start_node.x, start_node.y, start_node.z, x_pos, y_pos, z_pos, x_neg,
      y_neg, z_neg);

  ROS_INFO(
      "end_node_x=%f   y=%f  z=%f     x_pos=%f  y_pos=%f  z_pos=%f  x_neg=%f "
      " y_neg=%f  z_neg=%f",
      end_node.x, end_node.y, end_node.z, x_pos, y_pos, z_pos, x_neg, y_neg,
      z_neg);

  for (int i = 0; i < 6; i++) {
    ROS_INFO("border=%f", borders[i]);
  }
}

/*********************************************************/
/******************** Flight Corridor ********************/
/*********************************************************/

bool FlightCorridor::isCubeSafe(FlightCube cube,
                                std::vector<Eigen::Vector3i> &collision_grids,
                                int last_node_order, int check_order,
                                int print_flag) {
  //   return isOccupied(1,1,1);
  for (int i = cube.start_node.x - cube.x_neg_int;
       i <= cube.start_node.x + cube.x_pos_int; i++)
    for (int j = cube.start_node.y - cube.y_neg_int;
         j <= cube.start_node.y + cube.y_pos_int; j++)
      for (int k = cube.start_node.z - cube.z_neg_int;
           k <= cube.start_node.z + cube.z_pos_int; k++) {
        if (gridmap_->getInflateOccupancy(
                gridmap_->indexToPos(Eigen::Vector3i(i, j, k))))
        // if (isOccupied(i,j,k))
        {
          if (last_node_order == check_order - 1) {
            collision_grids.push_back(Eigen::Vector3i(i, j, k));
            // ROS_INFO("COLLISION_GRID_SIZE=%d",collision_grids.size());
          }
          if (print_flag)
            ROS_INFO(
                "collision with  %d  %d  %d     start_node_x=%d  y=%d  z=%d   "
                "end_node_x=%d  y=%d  z=%d",
                i, j, k, cube.start_node.x, cube.start_node.y,
                cube.start_node.z, cube.end_node.x, cube.end_node.y,
                cube.end_node.z);
          return 0;
        }
      }
  return 1;
}

/**
 * @brief
 * TODO: x, y, z coordinates is double, change this to int
 * @param cube
 * @param print_flag
 * @return true
 * @return false
 */
bool FlightCorridor::isCubeSafe(FlightCube cube, int print_flag) {
  //   return isOccupied(1,1,1);
  for (int i = cube.start_node.x - cube.x_neg_int;
       i <= cube.start_node.x + cube.x_pos_int; i++)
    for (int j = cube.start_node.y - cube.y_neg_int;
         j <= cube.start_node.y + cube.y_pos_int; j++)
      for (int k = cube.start_node.z - cube.z_neg_int;
           k <= cube.start_node.z + cube.z_pos_int; k++) {
        if (gridmap_->getInflateOccupancy(
                gridmap_->indexToPos(Eigen::Vector3i(i, j, k))))
        // if (isOccupied(i,j,k))
        {
          if (print_flag)
            ROS_INFO(
                "collision with  %d  %d  %d     start_node_x=%d  y=%d  z=%d   "
                "end_node_x=%d  y=%d  z=%d",
                i, j, k, cube.start_node.x, cube.start_node.y,
                cube.start_node.z, cube.end_node.x, cube.end_node.y,
                cube.end_node.z);
          return 0;
        }
      }
  return 1;
}

/**
 * @brief 
 * 
 * @param cube 
 * @return FlightCube 
 */
FlightCube FlightCorridor::expandCube(FlightCube &cube) {
  int max_expand_size = 4;
  int x_pos_origin = cube.x_pos_int;
  int x_neg_origin = cube.x_neg_int;
  int y_pos_origin = cube.y_pos_int;
  int y_neg_origin = cube.y_neg_int;
  int z_pos_origin = cube.z_pos_int;
  int z_neg_origin = cube.z_neg_int;

  bool at_least_one_suc_flag = 1;
  while (at_least_one_suc_flag) {
    at_least_one_suc_flag = 0;
    // for x
    if (cube.x_pos_int - x_pos_origin <= max_expand_size) {
      cube.x_pos_int++;
      if (!isCubeSafe(cube))
        cube.x_pos_int--;
      else
        at_least_one_suc_flag = 1;
    }

    if (cube.x_neg_int - x_neg_origin <= max_expand_size) {
      cube.x_neg_int++;
      if (!isCubeSafe(cube))
        cube.x_neg_int--;
      else
        at_least_one_suc_flag = 1;
    }

    // for y
    if (cube.y_pos_int - y_pos_origin <= max_expand_size) {
      cube.y_pos_int++;
      if (!isCubeSafe(cube))
        cube.y_pos_int--;
      else
        at_least_one_suc_flag = 1;
    }

    if (cube.y_neg_int - y_neg_origin <= max_expand_size) {
      cube.y_neg_int++;
      if (!isCubeSafe(cube))
        cube.y_neg_int--;
      else
        at_least_one_suc_flag = 1;
    }

    // for z
    if (cube.z_pos_int - z_pos_origin <= max_expand_size) {
      cube.z_pos_int++;
      if (!isCubeSafe(cube))
        cube.z_pos_int--;
      else
        at_least_one_suc_flag = 1;
    }

    if (cube.z_neg_int - z_neg_origin <= max_expand_size &&
        cube.start_node.z - cube.z_neg_int - 1 >= 0) {
      cube.z_neg_int++;
      if (!isCubeSafe(cube))
        cube.z_neg_int--;
      else
        at_least_one_suc_flag = 1;
    }

    // ROS_INFO("x= %d  %d  y=%d  %d   z=%d  %d
    // ",cube.x_pos_int,cube.x_neg_int,cube.y_pos_int,
    // cube.y_neg_int,cube.z_pos_int,cube.z_neg_int);
  }
  return cube;
  // for(int i=-max_expand_size;i<=max_expand_size;i++)
  //     for(int j=-max_expand_size;j<=max_expand_size;j++)
  //         for(int k=-max_expand_size;k<=max_expand_size;k++)
  //         {
  //             if(i>0)
  //             {
  //                 cube.x_pos_int+=i;
  //             }
  //             else
  //             {
  //                 cube.x_neg_int+=i;
  //             }

  //             if(i>0)
  //             {
  //                 cube.x_pos_int+=i;
  //             }
  //             else
  //             {
  //                 cube.x_neg_int+=i;
  //             }
  //         }
}

/**
 * @brief
 *
 * @param cube
 */
void FlightCorridor::updateAttributes(FlightCube &cube) {
  {
    Eigen::Vector3i temp_idx(cube.start_node.x - cube.x_neg_int,
                             cube.start_node.y - cube.y_neg_int,
                             cube.start_node.z - cube.z_neg_int);
    Eigen::Vector3d temp_coord = gridmap_->indexToPos(temp_idx);
    cube.x_neg = cube.start_node->position[0] - temp_coord[0] +
                 0.5 * gridmap_->getResolution();
    cube.y_neg = cube.start_node->position[1] - temp_coord[1] +
                 0.5 * gridmap_->getResolution();
    cube.z_neg = cube.start_node->position[2] - temp_coord[2] +
                 0.5 * gridmap_->getResolution();
    cube.borders[0] = cube.start_node->position[0] - cube.x_neg;
    cube.borders[2] = cube.start_node->position[1] - cube.y_neg;
    cube.borders[4] = cube.start_node->position[2] - cube.z_neg;
    cube.borders_int[0] = cube.start_node.x - cube.x_neg_int;
    cube.borders_int[2] = cube.start_node.y - cube.y_neg_int;
    cube.borders_int[4] = cube.start_node.z - cube.z_neg_int;
  }

  {
    Eigen::Vector3i temp_idx(cube.start_node.x + cube.x_pos_int,
                             cube.start_node.y + cube.y_pos_int,
                             cube.start_node.z + cube.z_pos_int);
    Eigen::Vector3d temp_coord = gridmap_->indexToPos(temp_idx);
    cube.x_pos = temp_coord[0] - cube.start_node->position[0] +
                 0.5 * gridmap_->getResolution();
    cube.y_pos = temp_coord[1] - cube.start_node->position[1] +
                 0.5 * gridmap_->getResolution();
    cube.z_pos = temp_coord[2] - cube.start_node->position[2] +
                 0.5 * gridmap_->getResolution();
    cube.borders[1] = cube.start_node->position[0] + cube.x_pos;
    cube.borders[3] = cube.start_node->position[1] + cube.y_pos;
    cube.borders[5] = cube.start_node->position[2] + cube.z_pos;
    cube.borders_int[1] = cube.start_node.x + cube.x_pos_int;
    cube.borders_int[3] = cube.start_node.y + cube.y_pos_int;
    cube.borders_int[5] = cube.start_node.z + cube.z_pos_int;
  }
}

/**
 * @brief divide the corridor to multiple sub corridors
 *
 * @param min_length
 */
void FlightCorridor::divideCorridor(double min_length) {
  std::vector<FlightCube> new_cubes;
  // ROS_INFO("CORRIDOR_SIZE=%d ",cubes.size());
  for (int i = 0; i < cubes.size(); i++) {
    // ROS_INFO("start=%f
    // end=%f",cubes[i].start_node->coord[0],cubes[i].end_node->coord[0]);
    Eigen::Vector3d diff_vec =
        cubes[i].end_node->position - cubes[i].start_node->position;
    if (diff_vec.norm() >= min_length * 2) {
      double divide_num = floor((diff_vec.norm() / min_length));
      // ROS_INFO("diff_vec_norm=%f   divide_num=%f"
      // ,diff_vec.norm(),divide_num);

      Eigen::Vector3d last_vec = cubes[i].start_node->position;
      NodePtr last_node = new Node();
      last_node->position = last_vec;
      last_node->index = gridmap_->posToIndex(last_vec);
      for (double j = 1; j <= divide_num; j++) {
        Eigen::Vector3d cur_vec =
            cubes[i].start_node->position + j / divide_num * diff_vec;
        NodePtr cur_node = new Node();
        cur_node->position = cur_vec;
        cur_node->index = gridmap_->posToIndex(cur_vec);
        FlightCube temp_cube(last_node, cur_node);
        expandCube(temp_cube);
        updateAttributes(temp_cube);
        new_cubes.push_back(temp_cube);
        last_node = cur_node;
      }
    } else {
      new_cubes.push_back(cubes[i]);
    }
  }
  cubes = new_cubes;
}

/**
 * @brief 
 * 
 * @return Eigen::MatrixXd 
 */
Eigen::MatrixXd FlightCorridor::corridor2mat() {
  // ROS_INFO("cubes.size()=%d",cubes.size());
  Eigen::MatrixXd out;
  out.resize(6, cubes.size());
  for (int i = 0; i < cubes.size(); i++) {
    Eigen::MatrixXd col;
    col.resize(6, 1);
    // col<<cubes[i].borders_int[0],cubes[i].borders_int[1],cubes[i].borders_int[2],
    // cubes[i].borders_int[3],cubes[i].borders_int[4],cubes[i].borders_int[5];
    // col<<cubes[i].borders[0],cubes[i].borders[1],cubes[i].borders[2],
    // cubes[i].borders[3],cubes[i].borders[4],cubes[i].borders[5];
    col << cubes[i].borders[0], cubes[i].borders[2], cubes[i].borders[4],
        cubes[i].borders[1], cubes[i].borders[3], cubes[i].borders[5];
    out.block<6, 1>(0, i) << col;
  }
  return out;
}
/**
 * @brief 
 * 
 * @param gridpath 
 * @return int 
 */
int FlightCorridor::generate(std::vector<Eigen::Vector3d> gridpath) {
  cubes.clear();
  Eigen::Vector3d Start_point = gridpath[0];
  Eigen::Vector3d End_point = gridpath[gridpath.size() - 1];

  if (gridmap_->getInflateOccupancy(Start_point)) {
    ROS_ERROR("SFC: Start point is not collision-free");
    return 0;
  }
  if (gridmap_->getInflateOccupancy(End_point)) {
    ROS_ERROR("end point is not collision-free");
    return 0;
  }
  Eigen::Vector3d cube_point1, cube_point2;
  int index1, index2;
  index1 = index2 = 0;
  int path_len = gridpath.size();
  bool suc_flag = 0;
  while (true) {
    int i;
    for (i = index2; i < path_len; i++) {
      cube_point1 = gridpath[index2];
      cube_point2 = gridpath[i];
      Eigen::Vector3i grid1 = gridmap_->posToIndex(cube_point1);
      Eigen::Vector3i grid2 = gridmap_->posToIndex(cube_point2);
      int x_min, x_max, y_min, y_max, z_min, z_max;
      x_min = min(grid1[0], grid2[0]);
      x_max = max(grid1[0], grid2[0]);
      y_min = min(grid1[1], grid2[1]);
      y_max = max(grid1[1], grid2[1]);
      z_min = min(grid1[2], grid2[2]);
      z_max = max(grid1[2], grid2[2]);
      int safe_flag = 1;
      for (int j = x_min; j <= x_max; j++) {
        for (int k = y_min; k <= y_max; k++) {
          for (int l = z_min; l <= z_max; l++) {
            if (gridmap_->getInflateOccupancy(
                    gridmap_->indexToPos(Eigen::Vector3i(j, k, l)))) {
              safe_flag = 0;
              break;
            }
          }
        }
      }
      if (!safe_flag) break;
    }
    index1 = index2;
    index2 = i - 1;
    NodePtr start_node = new Node();
    start_node->position = gridpath[index1];
    start_node->index = gridmap_->posToIndex(gridpath[index1]);
    NodePtr end_node = new Node();
    end_node->position = gridpath[index2];
    end_node->index = gridmap_->posToIndex(gridpath[index2]);
    FlightCube temp_cube(start_node, end_node);
    expandCube(temp_cube);
    updateAttributes(temp_cube);
    cubes.push_back(temp_cube);
    if (index2 >= path_len - 1) {
      break;
    } else if (index1 == index2) {
      ROS_ERROR("SFC: sfc generator died!!!");
      return 0;
    }
  }
  if (cubes.size() == 1) {
    double min_length =
        (cubes[0].start_node->position - cubes[0].end_node->position).norm() /
        2.1;
    divideCorridor(min_length);
  }
  return 1;
}
/**
 * @file safe_flight_corridor.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 *
 * @version 1.0
 * @date 2021-12-18
 *
 * Original created by Han-Sin in ZJU-FAST-Lab
 * Modified by Siyuan Wu to fit our P&DM project
 *
 */

#include <flight_corridor/safe_flight_corridor.h>

/*****************************************************/
/******************** Flight Cube ********************/
/*****************************************************/

// TODO: int or double? what does index means in Node class
FlightCube::FlightCube(const Eigen::Vector3i &start,
                       const Eigen::Vector3i &end) {
  start_node_ = start;
  end_node_ = end;
  if (end_node_[0] > start_node_[0]) {  // init x
    x_pos_int = end_node_[0] - start_node_[0];
    x_neg_int = 0;
  } else {
    x_neg_int = start_node_[0] - end_node_[0];
    x_pos_int = 0;
  }

  if (end_node_[1] > start_node_[1]) {  // init y
    y_pos_int = end_node_[1] - start_node_[1];
    y_neg_int = 0;
  } else {
    y_neg_int = start_node_[1] - end_node_[1];
    y_pos_int = 0;
  }

  if (end_node_[2] > start_node_[2]) {  // init z
    z_pos_int = end_node_[2] - start_node_[2];
    z_neg_int = 0;
  } else {
    z_neg_int = start_node_[2] - end_node_[2];
    z_pos_int = 0;
  }
}

void FlightCube::DisplayCube() {
  // ROS_INFO("start_node_x_int=%d   y_int=%d   z_int=%d     x_pos_int=%d
  // y_pos_int=%d  z_pos_int=%d  x_neg_int=%d  y_neg_int=%d  z_neg_int=%d",
  // start_node_[0],start_node_[1],start_node_[2],x_pos_int,y_pos_int,z_pos_int,x_neg_int,y_neg_int,z_neg_int);

  ROS_INFO(
      "start_node_x=%d   y=%d  z=%d     x_pos=%d  y_pos=%d  z_pos=%d  "
      "x_neg=%d  y_neg=%d  z_neg=%d",
      start_node_[0], start_node_[1], start_node_[2], x_pos, y_pos, z_pos,
      x_neg, y_neg, z_neg);

  ROS_INFO(
      "end_node_x=%d   y=%d  z=%d     x_pos=%d  y_pos=%d  z_pos=%d  x_neg=%d "
      " y_neg=%d  z_neg=%d",
      end_node_[0], end_node_[1], end_node_[2], x_pos, y_pos, z_pos, x_neg,
      y_neg, z_neg);

  for (int i = 0; i < 6; i++) {
    ROS_INFO("border=%d", borders[i]);
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
  for (int i = cube.start_node_[0] - cube.x_neg_int;
       i <= cube.start_node_[0] + cube.x_pos_int; i++)
    for (int j = cube.start_node_[1] - cube.y_neg_int;
         j <= cube.start_node_[1] + cube.y_pos_int; j++)
      for (int k = cube.start_node_[2] - cube.z_neg_int;
           k <= cube.start_node_[2] + cube.z_pos_int; k++) {
        if (gridmap_->isPointCollision(Eigen::Vector3i(i, j, k))) {
          if (last_node_order == check_order - 1) {
            collision_grids.push_back(Eigen::Vector3i(i, j, k));
            // ROS_INFO("COLLISION_GRID_SIZE=%d",collision_grids.size());
          }
          if (print_flag)
            ROS_INFO(
                "collision with  %d  %d  %d     start_node_x=%d  y=%d  z=%d   "
                "end_node_x=%d  y=%d  z=%d",
                i, j, k, cube.start_node_[0], cube.start_node_[1],
                cube.start_node_[2], cube.end_node_[0], cube.end_node_[1],
                cube.end_node_[2]);
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
  for (int i = cube.start_node_[0] - cube.x_neg_int;
       i <= cube.start_node_[0] + cube.x_pos_int; i++)
    for (int j = cube.start_node_[1] - cube.y_neg_int;
         j <= cube.start_node_[1] + cube.y_pos_int; j++)
      for (int k = cube.start_node_[2] - cube.z_neg_int;
           k <= cube.start_node_[2] + cube.z_pos_int; k++) {
        if (gridmap_->isPointCollision(Eigen::Vector3i(i, j, k))) {
          if (print_flag)
            ROS_INFO(
                "collision with  %d  %d  %d     start_node_x=%d  y=%d  z=%d   "
                "end_node_x=%d  y=%d  z=%d",
                i, j, k, cube.start_node_[0], cube.start_node_[1],
                cube.start_node_[2], cube.end_node_[0], cube.end_node_[1],
                cube.end_node_[2]);
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
  /* what is max_expand_size ??? */
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
        cube.start_node_[2] - cube.z_neg_int - 1 >= 0) {
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
    Eigen::Vector3i temp_idx(cube.start_node_[0] - cube.x_neg_int,
                             cube.start_node_[1] - cube.y_neg_int,
                             cube.start_node_[2] - cube.z_neg_int);
    Eigen::Vector3d temp_coord = gridmap_->indexToPos(temp_idx);
    cube.x_neg = cube.start_node_[0] - temp_coord[0] +
                 0.5 * gridmap_->getResolution();
    cube.y_neg = cube.start_node_[1] - temp_coord[1] +
                 0.5 * gridmap_->getResolution();
    cube.z_neg = cube.start_node_[2] - temp_coord[2] +
                 0.5 * gridmap_->getResolution();
    cube.borders[0] = cube.start_node_[0] - cube.x_neg;
    cube.borders[2] = cube.start_node_[1] - cube.y_neg;
    cube.borders[4] = cube.start_node_[2] - cube.z_neg;
    cube.borders_int[0] = cube.start_node_[0] - cube.x_neg_int;
    cube.borders_int[2] = cube.start_node_[1] - cube.y_neg_int;
    cube.borders_int[4] = cube.start_node_[2] - cube.z_neg_int;
  }

  {
    Eigen::Vector3i temp_idx(cube.start_node_[0] + cube.x_pos_int,
                             cube.start_node_[1] + cube.y_pos_int,
                             cube.start_node_[2] + cube.z_pos_int);
    Eigen::Vector3d temp_coord = gridmap_->indexToPos(temp_idx);
    cube.x_pos = temp_coord[0] - cube.start_node_[0] +
                 0.5 * gridmap_->getResolution();
    cube.y_pos = temp_coord[1] - cube.start_node_[1] +
                 0.5 * gridmap_->getResolution();
    cube.z_pos = temp_coord[2] - cube.start_node_[2] +
                 0.5 * gridmap_->getResolution();
    cube.borders[1] = cube.start_node_[0] + cube.x_pos;
    cube.borders[3] = cube.start_node_[1] + cube.y_pos;
    cube.borders[5] = cube.start_node_[2] + cube.z_pos;
    cube.borders_int[1] = cube.start_node_[0] + cube.x_pos_int;
    cube.borders_int[3] = cube.start_node_[1] + cube.y_pos_int;
    cube.borders_int[5] = cube.start_node_[2] + cube.z_pos_int;
  }
}

/**
 * @brief divide the corridor to multiple sub corridors
 * TODO: edit this to fit our vertices class
 * @param min_length
 */
void FlightCorridor::divideCorridor(double min_length) {
  std::vector<FlightCube> new_cubes;
  ROS_INFO("CORRIDOR_SIZE=%d ", cubes_.size());
  for (int i = 0; i < cubes_.size(); i++) {
    ROS_INFO("start=%d \nend=%d", cubes_[i].start_node_[0], cubes_[i].end_node_[0]);

    Eigen::Vector3d diff_vec =
       gridmap_->indexToPos(cubes_[i].end_node_) -
       gridmap_->indexToPos(cubes_[i].start_node_);

    if (diff_vec.norm() >= min_length * 2) {
      double divide_num = floor((diff_vec.norm() / min_length));
      // ROS_INFO("diff_vec_norm=%d   divide_num=%d"
      // ,diff_vec.norm(),divide_num);

      Eigen::Vector3d last_vec = gridmap_->indexToPos(cubes_[i].start_node_);
      Eigen::Vector3i last_node = cubes_[i].start_node_;

      for (double j = 1; j <= divide_num; j++) {
        Eigen::Vector3d cur_vec =
            gridmap_->indexToPos(cubes_[i].start_node_) +
            j / divide_num * diff_vec;
        Eigen::Vector3i cur_node = gridmap_->posToIndex(cur_vec);
        FlightCube temp_cube(last_node, cur_node);

        expandCube(temp_cube);
        updateAttributes(temp_cube);
        new_cubes.push_back(temp_cube);
        last_node = cur_node;
      }
    } else {
      new_cubes.push_back(cubes_[i]);
    }
  }
  cubes_ = new_cubes;
}

/**
 * @brief
 *
 * @return Eigen::MatrixXd
 */
Eigen::MatrixXd FlightCorridor::corridor2mat() {
  // ROS_INFO("cubes.size()=%d",cubes.size());
  Eigen::MatrixXd out;
  out.resize(6, cubes_.size());
  for (int i = 0; i < cubes_.size(); i++) {
    Eigen::MatrixXd col;
    col.resize(6, 1);
    // col<<cubes_[i].borders_int[0],cubes_[i].borders_int[1],cubes_[i].borders_int[2],
    // cubes_[i].borders_int[3],cubes_[i].borders_int[4],cubes_[i].borders_int[5];
    // col<<cubes_[i].borders[0],cubes_[i].borders[1],cubes_[i].borders[2],
    // cubes_[i].borders[3],cubes_[i].borders[4],cubes_[i].borders[5];
    col << cubes_[i].borders[0], cubes_[i].borders[2], cubes_[i].borders[4],
        cubes_[i].borders[1], cubes_[i].borders[3], cubes_[i].borders[5];
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
int FlightCorridor::generate(std::vector<Eigen::Vector3d> &gridpath) {
  cubes_.clear();
  Eigen::Vector3d Start_point = gridpath[0];
  Eigen::Vector3d End_point = gridpath[gridpath.size() - 1];

  if (gridmap_->isPointCollision(Start_point)) {
    ROS_ERROR("SFC: Start point is not collision-free");
    return 0;
  }
  if (gridmap_->isPointCollision(End_point)) {
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
      x_min = std::min(grid1[0], grid2[0]);
      x_max = std::max(grid1[0], grid2[0]);
      y_min = std::min(grid1[1], grid2[1]);
      y_max = std::max(grid1[1], grid2[1]);
      z_min = std::min(grid1[2], grid2[2]);
      z_max = std::max(grid1[2], grid2[2]);
      int safe_flag = 1;
      for (int j = x_min; j <= x_max; j++) {
        for (int k = y_min; k <= y_max; k++) {
          for (int l = z_min; l <= z_max; l++) {
            if (gridmap_->isPointCollision(Eigen::Vector3i(j, k, l))) {
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

    Eigen::Vector3i start_node_ = gridmap_->posToIndex(gridpath[index1]);
    Eigen::Vector3i end_node_ = gridmap_->posToIndex(gridpath[index2]);
    FlightCube temp_cube(start_node_, end_node_);

    expandCube(temp_cube);
    updateAttributes(temp_cube);
    cubes_.push_back(temp_cube);

    if (index2 >= path_len - 1) {
      break;
    } else if (index1 == index2) {
      ROS_ERROR("SFC: sfc generator died!!!");
      return 0;
    }
  }
  if (cubes_.size() == 1) {
    Eigen::Vector3d start_pos = gridmap_->indexToPos(cubes_[0].start_node_);
    Eigen::Vector3d end_pos = gridmap_->indexToPos(cubes_[0].end_node_);
    double min_length = (start_pos - end_pos).norm() / 2.1;
    divideCorridor(min_length);
  }
  return 1;
}
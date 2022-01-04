/**
 * @file grid_map.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2021-11-20
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "map_server/grid_map.h"

void GridMap::initGridMap(ros::NodeHandle& nh) {
  /* read parameters */
  nh.param("/grid_map/resolution", _mp_resolution, 0.05f);
  nh.param("/grid_map/frame_id", _mp_frame_id, std::string("world"));
  nh.param("/grid_map/obstacle_inflation", _mp_inflation, 0.2f);
  nh.param("/grid_map/x_size", _mp_size_x, -1.0f);
  nh.param("/grid_map/y_size", _mp_size_y, -1.0f);
  nh.param("/grid_map/z_size", _mp_size_z, -1.0f);
  // /* to debug
  // nh.param("/grid_map/x_size", _mp_size_x, 26.0f);
  // nh.param("/grid_map/y_size", _mp_size_y, 20.0f);
  // nh.param("/grid_map/z_size", _mp_size_z, 3.0f);

  ROS_INFO("Initializing GridMap Parameters:");
  ROS_INFO_STREAM("resolution:\t" << _mp_resolution);
  ROS_INFO_STREAM("obstacle_inflation:\t" << _mp_inflation);
  ROS_INFO_STREAM("frame_id:\t" << _mp_frame_id);
  ROS_INFO_STREAM("map_size_x:\t" << _mp_size_x);
  ROS_INFO_STREAM("map_size_y:\t" << _mp_size_y);
  ROS_INFO_STREAM("map_size_z:\t" << _mp_size_z);

  /* calculate parameters */
  _mp_resolution_inv = 1 / _mp_resolution;
  _mp_origin_position = Eigen::Vector3f(-_mp_size_x / 2, -_mp_size_y / 2, -1);
  _inflate_size = ceil(_mp_inflation * _mp_resolution_inv);
  _inflate_size_z = 2;
  _mp_grid_size_x = _mp_size_x * _mp_resolution_inv;
  _mp_grid_size_y = _mp_size_y * _mp_resolution_inv;
  _mp_grid_size_z = (_mp_size_z + 1) * _mp_resolution_inv;

  /* subscriber */
  // cld_sub_ = nh.subscribe<sensor_msgs::PointCloud2>(
  //     "cloud_in", 1, &GridMap::pointCloudCallback, this);

  map_pub_ = nh.advertise<sensor_msgs::PointCloud2>("occupancy_inflate", 10);

  /* initialize */
  initBuffer(_mp_grid_size_x, _mp_grid_size_y, _mp_grid_size_z);
  ROS_INFO("GridMap Buffer initialized");

  /* map to be build */
  is_map_built_ = false;
}

/**
 * @brief read point cloud data and save to occupancy buffer
 *
 * @param cld
 */
void GridMap::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cld) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*cld, cloud);
  ROS_INFO("Received points clouds");

  int cloud_size = cloud.points.size();
  ROS_ERROR_COND(cloud_size == 0, "No points in this point cloud.");

  initBuffer(_mp_grid_size_x, _mp_grid_size_y, _mp_grid_size_z);
  // ROS_INFO("GridMap Buffer cleaned");
  // ROS_INFO_STREAM("x\t" << _mp_grid_size_x
  //  << "\ny\t" << _mp_grid_size_y
  //  << "\nz\t" << _mp_grid_size_z);

  pcl::PointXYZ pt;

  for (size_t i = 0; i < cloud_size; i++) {
    pt = cloud.points[i];
    Eigen::Vector3f p3f;

    if (pt.z < 0.0f) {
      continue;
    }

    /* traverse all grids and check if it is occupied */
    for (int x = -_inflate_size; x <= _inflate_size; ++x) {
      for (int y = -_inflate_size; y <= _inflate_size; ++y) {
        for (int z = -_inflate_size_z; z <= _inflate_size_z; ++z) {
          p3f(0) = pt.x + x * _mp_resolution;
          p3f(1) = pt.y + y * _mp_resolution;
          p3f(2) = pt.z + z * _mp_resolution;
          // ROS_INFO_STREAM("x\t" << p3f(0) << "\ty\t" << p3f(1) << "\tz\t" <<
          // p3f(2));
          Eigen::Vector3i p_idx;
          posToIndex(p3f, p_idx);
          if (isIndexWithinBound(p_idx)) {
            // ROS_INFO_STREAM("x\t" << p_idx(0) << "\ty\t" << p_idx(1) <<
            // "\tz\t" << p_idx(2));
            occupancy_buffer_[indexToAddress(p_idx)] = true;
          } else {
            continue;
          }
        }
      }
    }
  }
  // ROS_INFO_STREAM("Occupancy size");
  publish();
}

/**
 * @brief publish grid map as point clouds
 *
 */
void GridMap::publish() {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointXYZ pt;
  for (int x = 0; x < _mp_grid_size_x; x++) {
    for (int y = 0; y < _mp_grid_size_y; y++) {
      for (int z = 0; z < _mp_grid_size_z; z++) {
        if (occupancy_buffer_[indexToAddress(x, y, z)]) {
          Eigen::Vector3f pos;
          Eigen::Vector3i idx(x, y, z);
          indexToPos(idx, pos);
          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          // ROS_INFO_STREAM(pt.x << " " << pt.y << " " << pt.z);
          cloud.push_back(pt);
        }
      }
    }
  }
  ROS_INFO_STREAM("publish size " << cloud.points.size());
  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = _mp_frame_id;
  sensor_msgs::PointCloud2 cloud_msgs;
  pcl::toROSMsg(cloud, cloud_msgs);
  map_pub_.publish(cloud_msgs);
  ROS_INFO("GridMap publish complete");
  is_map_built_ = true;
}

/**
 * @brief initialize occupancy buffer as free space (no obstacle)
 *
 * @param grid_size_x maximum number of grids along x axis
 * @param grid_size_y maximum number of grids along y axis
 * @param grid_size_z maximum number of grids along z axis
 */
void GridMap::initBuffer(int grid_size_x, int grid_size_y, int grid_size_z) {
  occupancy_buffer_.resize(grid_size_x * grid_size_y * grid_size_z);
  for (auto it = occupancy_buffer_.begin(); it != occupancy_buffer_.end();
       it++) {
    *it = false;
  }
  // for (int x = 0; x < grid_size_x; x++) {
  //   for (int y = 0; y < grid_size_y; y++) {
  //     for (int z = 0; z < grid_size_z; z++) {
  //       occupancy_buffer_[indexToAddress(x, y, z)] = false;
  //     }
  //   }
  // }
}

/**
 * @brief convert real world position to grid position
 * @param pos 3d points in real world, float
 * @param id return, 3d points in grid map, int
 */
inline void GridMap::posToIndex(const Eigen::Vector3f& pos,
                                Eigen::Vector3i& id) {
  for (int i = 0; i < 3; i++) {
    id(i) = static_cast<int>(
        floor((pos(i) - _mp_origin_position(i)) * _mp_resolution_inv));
  }
}

/**
 * @brief convert real world position to grid position
 * @param pos 3d points in real world, float
 * @return Eigen::Vector3i 3d points in grid map, int
 */
Eigen::Vector3i GridMap::posToIndex(const Eigen::Vector3f& pos) {
  Eigen::Vector3i id;
  for (int i = 0; i < 3; i++) {
    id(i) = static_cast<int>(
        floor((pos(i) - _mp_origin_position(i)) * _mp_resolution_inv));
  }
  return id;
}

/**
 * @brief convert real world position to grid position
 * @param pos 3d points in real world, double
 * @return Eigen::Vector3i 3d points in grid map, int
 */
Eigen::Vector3i GridMap::posToIndex(const Eigen::Vector3d& pos) {
  Eigen::Vector3i id;
  for (int i = 0; i < 3; i++) {
    id(i) = static_cast<int>(
        floor((pos(i) - _mp_origin_position(i)) * _mp_resolution_inv));
  }
  return id;
}

/**
 * @brief convert real world position to address in buffer
 * @param pos
 * @return int
 */
inline int GridMap::posToAddress(const Eigen::Vector3f& pos) {
  Eigen::Vector3i ipos;
  posToIndex(pos, ipos);
  int address = indexToAddress(ipos);
  return address;
}

/**
 * @brief convert grid position to address in buffer
 *
 * @param id grid position
 * @return int address
 */
inline int GridMap::indexToAddress(const Eigen::Vector3i& id) {
  return id(0) * _mp_grid_size_y * _mp_grid_size_z + id(1) * _mp_grid_size_z +
         id(2);
}

inline int GridMap::indexToAddress(int& x, int& y, int& z) {
  return x * _mp_grid_size_y * _mp_grid_size_z + y * _mp_grid_size_z + z;
}

/**
 * @brief convert grid position to real world coordinates
 *
 * @param idx 3d points in grid map, int
 * @param pos return, 3d coordinates in real world, float
 */
inline void GridMap::indexToPos(const Eigen::Vector3i& idx,
                                Eigen::Vector3f& pos) {
  for (int i = 0; i < 3; i++) {
    pos(i) = (idx(i) + float(0.5)) * _mp_resolution + _mp_origin_position(i);
    // std::cout << "i" << pos(i) << std::endl;
  }
}
/**
 * @brief convert grid position to real world coordinates
 * @param idx 3d points in grid map, int
 * @return Eigen::Vector3d, 3d coordinates in real world, double
 */
Eigen::Vector3d GridMap::indexToPos(const Eigen::Vector3i& idx) {
  Eigen::Vector3d pos;
  for (int i = 0; i < 3; i++) {
    pos(i) = (idx(i) + 0.5) * _mp_resolution + _mp_origin_position(i);
    // std::cout << "i" << pos(i) << std::endl;
  }
  return pos;
}

inline Eigen::Vector3f GridMap::indexToPos(const Eigen::Vector3i& idx) {
  Eigen::Vector3f pos;
  for (int i = 0; i < 3; i++) {
    pos(i) = (idx(i) + float(0.5)) * _mp_resolution + _mp_origin_position(i);
  }
  return pos;
}

inline Eigen::Vector3f GridMap::indexToPos(int& x, int& y, int& z) {
  Eigen::Vector3i index(x, y, z);
  return indexToPos(index);
}

/**
 * @brief check if index exceeds the boundary of grid map
 * @param idx index to be checked
 * @return true  : within boundary
 * @return false : exceed boundary
 */
inline bool GridMap::isIndexWithinBound(const Eigen::Vector3i idx) {
  return idx(0) >= 0 && idx(1) >= 0 && idx(2) >= 0 &&
         idx(0) < _mp_grid_size_x && idx(1) < _mp_grid_size_y &&
         idx(2) < _mp_grid_size_z;
}

inline bool GridMap::isPosWithinBound(const Eigen::Vector3f p) {
  Eigen::Vector3i idx;
  posToIndex(p, idx);
  return isIndexWithinBound(idx);
}

/**
 * @brief
 *
 * @param p points, float
 * @return true collide
 * @return false not collide
 */
bool GridMap::isPointCollision(const Eigen::Vector3f& p) {
  if (!isPosWithinBound(p)) {
    return false;
  }
  return occupancy_buffer_[posToAddress(p)];
}

bool GridMap::isPointCollision(int& x, int& y, int& z) {
  if (!isPosWithinBound(indexToPos(x, y, z))) {
    return false;
  }
  return occupancy_buffer_[indexToAddress(x, y, z)];
}

/**
 * @brief
 *
 * @param p points, double
 * @return true collide
 * @return false not collide
 */
bool GridMap::isPointCollision(const Eigen::Vector3d& p) {
  Eigen::Vector3f pf = p.cast<float>();
  if (!isPosWithinBound(pf)) {
    return false;
  }
  return occupancy_buffer_[posToAddress(pf)];
}

/**
 * @brief
 *
 * @param i index, int
 * @return true collide
 * @return false not collide
 */
bool GridMap::isPointCollision(const Eigen::Vector3i& i) {
  if (!isIndexWithinBound(i)) {
    return false;
  }
  return occupancy_buffer_[indexToAddress(i)];
}

/**
 * @brief check straight line collision by sampling along the line
 *
 * @param start start point of the straight
 * @param end   end point of the straight
 * @return true
 * @return false collision-free path
 */
bool GridMap::isStraightLineCollision(const Eigen::Vector3f& start,
                                      const Eigen::Vector3f& end) {
  bool isCollision = false;
  float sample_ratio = 0.5f;
  float resolution = 0.1f;

  float Dx = end(0) - start(0);
  float Dy = end(1) - start(1);
  float Dz = end(2) - start(2);
  float D = sqrt(Dx * Dx + Dy * Dy + Dz * Dz);
  float dx = Dx / D * resolution * sample_ratio;
  float dy = Dy / D * resolution * sample_ratio;
  float dz = Dz / D * resolution * sample_ratio;
  // float dx = Dx / 100;
  // float dy = Dy / 100;
  // float dz = Dz / 100;

  float sum_x = 0.0f;

  float curr_x = start(0);
  float curr_y = start(1);
  float curr_z = start(2);

  while (abs(sum_x) < abs(Dx)) {
    sum_x += dx;

    curr_x += dx;
    curr_y += dy;
    curr_z += dz;
    Eigen::Vector3f curr(curr_x, curr_y, curr_z);
    Eigen::Vector3i curr_idx;
    posToIndex(curr, curr_idx);

    if (!isIndexWithinBound(curr_idx)) {
      continue;
    } else {
      if (occupancy_buffer_[indexToAddress(curr_idx)]) {
        // std::cout << "Found Collsion path" << std::endl;
        isCollision = true;
        return isCollision;
      }
    }
  }
  return isCollision;
}

/**
 * @brief check if building grid map finished
 * @return true    Grid map has built
 * @return false   Grid map is still building
 */
bool GridMap::isMapBuilt() { return is_map_built_; }

/**
 * @brief get map resolution
 * @return float
 */
float GridMap::getResolution() { return _mp_resolution; }

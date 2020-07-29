/*
 *  Copyright (C) 2020 Badger Technologies, LLC
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef AMCL_MAP_OCTOMAP_H
#define AMCL_MAP_OCTOMAP_H

#include <cstdint>
#include <limits>
#include <memory>
#include <queue>
#include <vector>

#include <octomap/OcTree.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>

#include "map/map.h"

namespace badger_amcl
{

class CachedDistanceOctoMap
{
public:
  CachedDistanceOctoMap(double resolution, double max_dist);

  std::vector<std::vector<std::vector<double>>> cached_distances_lut_;
  double resolution_;
  double max_dist_;
  int cell_radius_;
};

struct Index3 {
  Eigen::Vector3i v;
  inline bool operator< (const Index3& e) const
  {
    return v[0] != e.v[0] ? v[0] < e.v[0] : v[1] != e.v[1] ? v[1] < e.v[1] : v[2] < e.v[2];
  }
};


class OctoMap : public Map
{
public:
  OctoMap(double resolution);
  OctoMap(double resolution, bool publish_distances_lut);
  virtual ~OctoMap() = default;
  virtual void initFromOctree(std::shared_ptr<octomap::OcTree> octree, double max_distance_to_object);
  // Convert from map index to world coords
  virtual void convertMapToWorld(const std::vector<int>& map_coords,
                                 std::vector<double>* world_coords);
  // Convert from world coords to map coords
  virtual void convertWorldToMap(const std::vector<double>& world_coords,
                                 std::vector<int>* map_coords);
  // Test to see if the given map coords lie within the absolute map bounds.
  virtual bool isPoseValid(const int i, const int j);
  virtual bool isVoxelValid(const int i, const int j, const int k);
  virtual void getMinMaxCells(std::vector<int>* min_cells, std::vector<int>* max_cells);
  virtual void setMapBounds(const std::vector<double>& map_min, const std::vector<double>& map_max);
  // Update the distance values
  virtual void updateDistancesLUT();
  virtual double getMaxDistanceToObject();
  // This function is called very frequently.
  // Do not make it virtual as this would hinder performance.
  double getDistanceToObject(int i, int j, int k);

protected:
  const std::vector<std::vector<int>> SHIFTS = {{-1, 0, 0}, {0, -1, 0}, {0, 0, -1}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  struct OctoMapCellData;

  using CellDataQueue = std::queue<OctoMapCellData>;
  static constexpr double EPSILON = std::numeric_limits<double>::epsilon();

  virtual void iterateObstacleCells(CellDataQueue& q);
  virtual void iterateEmptyCells(CellDataQueue& q);
  virtual void enqueue(const int shift_index, const OctoMapCellData& current_cell, CellDataQueue& q);
  virtual inline uint32_t makePoseIndex(int i, int j);
  virtual inline void setDistanceToObject(int i, int j, int k, double d);

  std::shared_ptr<octomap::OcTree> octree_;
  std::vector<uint32_t> pose_indices_;
  std::vector<uint8_t> distance_ratios_;
  // Map dimensions (number of cells)
  std::vector<double> map_min_bounds_, map_max_bounds_;
  std::vector<int> cropped_min_cells_, cropped_max_cells_;
  CachedDistanceOctoMap cdm_;
  int map_cells_width_;
  uint32_t num_poses_;
  int num_z_column_indices_;
  double max_distance_ratio_;

  struct OctoMapCellData
  {
    OctoMapCellData() = default;
    int i, j, k;
    int src_i, src_j, src_k;
  };

private:
  void publishDistancesLUT();

  ros::NodeHandle nh_;
  ros::Publisher distances_lut_pub_;
  bool publish_distances_lut_;
};
}  // namespace amcl

#endif  // AMCL_MAP_OCTOMAP_H

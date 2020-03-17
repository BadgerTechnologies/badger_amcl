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
/*****************************************************
 * Desc: Octomap for 3D AMCL
 *****************************************************/

#ifndef AMCL_MAP_OCTOMAP_H
#define AMCL_MAP_OCTOMAP_H

#include <limits>
#include <memory>
#include <queue>
#include <vector>

#include <octomap/OcTree.h>
#include <tsl/sparse_map.h>

#include "map/map.h"

namespace amcl
{

class CachedDistanceOctoMap
{
public:
  CachedDistanceOctoMap(double resolution, double max_dist);

  std::vector<std::vector<std::vector<double>>> distances_;
  double resolution_;
  double max_dist_;
  int cell_radius_;
};

struct OctoMapCellData;

class OctoMap : public Map
{
public:
  OctoMap(double resolution, bool wait_for_occupancy_map);
  virtual ~OctoMap() {};
  // Convert from map index to world coords
  virtual void convertMapToWorld(const std::vector<int>& map_coords,
                                 std::vector<double>* world_coords);
  // Convert from world coords to map coords
  virtual void convertWorldToMap(const std::vector<double>& world_coords,
                                 std::vector<int>* map_coords);
  // Test to see if the given map coords lie within the absolute map bounds.
  virtual bool isValid(const std::vector<int>& coords);
  virtual std::vector<int> getSize();
  virtual void getMinMaxCells(std::vector<int>* min_cells, std::vector<int>* max_cells);
  virtual void setMapBounds(std::shared_ptr<std::vector<double>> map_min,
                            std::shared_ptr<std::vector<double>> map_max);
  // Update the cspace distance values
  virtual void updateCSpace();
  virtual void updateMaxOccDist(double max_occ_dist);
  virtual void initFromOctree(std::shared_ptr<octomap::OcTree> octree);
  virtual double getOccDist(int i, int j, int k);
  virtual double getMaxOccDist();

protected:
  static constexpr double EPSILON = std::numeric_limits<double>::epsilon();

  void iterateObstacleCells();
  void iterateEmptyCells();
  void updateNode(int i, int j, int k, const OctoMapCellData& current_cell);
  void setOccDist(int i, int j, int k, double d);
  bool enqueue(int i, int j, int k, int src_i, int src_j, int src_k);
  unsigned int computeCellIndex(int i, int j, int k);
  size_t makeHash(int i, int j, int k);

  std::shared_ptr<octomap::OcTree> octree_;
  tsl::sparse_map<size_t, double> distances_;
  tsl::sparse_map<size_t, double>::iterator distances_end_;
  tsl::sparse_map<size_t, double>::iterator hashmap_iterator_;
  // Map dimensions (number of cells)
  std::vector<double> map_min_bounds_, map_max_bounds_;
  std::vector<int> cropped_min_cells_, cropped_max_cells_, full_cells_;
  bool wait_for_occupancy_map_;
  CachedDistanceOctoMap cdm_;
  std::priority_queue<OctoMapCellData> q_;
  octomap::OcTree marked_;
};

struct OctoMapCellData
{
  OctoMap* octo_map;
  OctoMapCellData(OctoMap& o_map) : octo_map(&o_map) {}
  int i, j, k;
  int src_i, src_j, src_k;
  inline bool operator<(const OctoMapCellData& b) const
  {
    return octo_map->getOccDist(i, j, k) > b.octo_map->getOccDist(b.i, b.j, b.k);
  }
};
}  // namespace amcl

#endif  // AMCL_MAP_OCTOMAP_H

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
 * Author: Tyler Buchman (tyler_buchman@jabil.com)
 *****************************************************/

#ifndef AMCL_MAP_OCTOMAP_H
#define AMCL_MAP_OCTOMAP_H

#include "map/map.h"

#include <octomap/OcTree.h>
#include <octomap/OcTreeDistance.h>

#include <limits>
#include <memory>
#include <queue>
#include <vector>

namespace amcl
{
class OctoMap : public Map
{
public:
  OctoMap(bool wait_for_occupancy_map);
  // Convert from map index to world coords
  void convertMapToWorld(const std::vector<int>& map_coords, std::vector<double>* world_coords);
  // Convert from world coords to map coords
  void convertWorldToMap(const std::vector<double>& world_coords, std::vector<int>* map_coords);
  // Test to see if the given map coords lie within the absolute map bounds.
  bool isValid(const std::vector<int>& coords);
  std::vector<int> getSize();
  void getMinMaxCells(std::vector<int>* min_cells, std::vector<int>* max_cells);
  std::vector<double> getOrigin();
  void setOrigin(const std::vector<double>& origin);
  void setMapBounds(std::shared_ptr<std::vector<double>> map_min,
                    std::shared_ptr<std::vector<double>> map_max);
  // Update the cspace distance values
  void updateCSpace();
  void updateMaxOccDist(double max_occ_dist);
  void initFromOctree(std::shared_ptr<octomap::OcTree> octree, double lidar_height);
  double getOccDist(int i, int j, int k);
  double getOccDist(int i, int j);
  double getMaxOccDist();

private:
  static constexpr double EPSILON = std::numeric_limits<double>::epsilon();
  struct CellData
  {
    OctoMap* octo_map;
    CellData(OctoMap& o_map) : octo_map(&o_map)
    {
    }
    int i, j, k;
    int src_i, src_j, src_k;
  };

  class CachedDistanceMap
  {
  public:
    std::vector<std::vector<std::vector<double>>> distances_;
    double resolution_;
    double max_dist_;
    int cell_radius_;

    CachedDistanceMap(double resolution, double max_dist) : resolution_(resolution), max_dist_(max_dist)
    {
      cell_radius_ = max_dist / resolution;
      distances_.resize(cell_radius_ + 2);
      for (int i = 0; i <= cell_radius_ + 1; i++)
      {
        distances_[i].resize(cell_radius_ + 2);
        for (int j = 0; j <= cell_radius_ + 1; j++)
        {
          distances_[i][j].resize(cell_radius_ + 2);
          for (int k = 0; k <= cell_radius_ + 1; k++)
          {
            distances_[i][j][k] = sqrt(i * i + j * j + k * k);
          }
        }
      }
    }
  };

  void setOccDist(int i, int j, int k, double d);
  bool enqueue(int i, int j, int k, int src_i, int src_j, int src_k, std::priority_queue<CellData>& Q);
  unsigned int computeCellIndex(int i, int j, int k);

  friend bool operator<(const OctoMap::CellData& a, const OctoMap::CellData& b);

  std::shared_ptr<octomap::OcTree> octree_;
  std::unique_ptr<octomap::OcTreeDistance> distances_;
  // Map origin; the map is a viewport onto a conceptual larger map.
  std::vector<double> origin_;
  // Map dimensions (number of cells)
  std::vector<double> map_min_bounds_, map_max_bounds_;
  std::vector<int> cropped_min_cells_, cropped_max_cells_, full_cells_;
  double lidar_height_;
  bool wait_for_occupancy_map_;
  std::unique_ptr<CachedDistanceMap> cdm_;
};

inline bool operator<(const OctoMap::CellData& a, const OctoMap::CellData& b)
{
  return a.octo_map->getOccDist(a.i, a.j, a.k) > b.octo_map->getOccDist(b.i, b.j, b.k);
}
}  // namespace amcl

#endif  // AMCL_MAP_OCTOMAP_H

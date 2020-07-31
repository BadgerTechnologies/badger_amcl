/*
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
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

#ifndef AMCL_MAP_OCCUPANCY_MAP_H
#define AMCL_MAP_OCCUPANCY_MAP_H

#include <cmath>
#include <cstdint>
#include <memory>
#include <queue>
#include <vector>

#include "map/map.h"

namespace badger_amcl
{

// Description for a single map cell.
enum MapCellState
{
  CELL_FREE = -1,
  CELL_UNKNOWN = 0,
  CELL_OCCUPIED = 1
};

class CachedDistanceOccupancyMap
{
public:
  CachedDistanceOccupancyMap(double resolution, double max_dist);

  std::vector<std::vector<double>> cached_distances_lut_;
  double resolution_;
  double max_dist_;
  int cell_radius_;
};

class OccupancyMap : public Map
{
public:
  OccupancyMap(double resolution);
  virtual ~OccupancyMap() = default;
  // Convert from map index to world coords
  virtual void convertMapToWorld(const std::vector<int>& map_coords,
                                 std::vector<double>* world_coords);
  // Convert from world coords to map coords
  virtual void convertWorldToMap(const std::vector<double>& world_coords,
                                 std::vector<int>* map_coords);
  // Test to see if the given map coords lie within the absolute map bounds.
  virtual bool isValid(const std::vector<int>& coords);
  virtual void setOrigin(const pcl::PointXYZ& origin);
  virtual std::vector<int> getSize();
  virtual void setSize(std::vector<int> size_vec);
  // Update the distance values
  virtual void updateDistancesLUT(double max_distance_to_object);
  // Extract a single range reading from the map
  virtual double calcRange(double ox, double oy, double oa, double max_range);
  // Compute the cell index for the given map coords.
  virtual unsigned int computeCellIndex(int i, int j);
  virtual double getMaxDistanceToObject();
  virtual MapCellState getCellState(int i, int j);
  virtual void setCellState(int index, MapCellState state);
  // This function is called very frequently.
  // Do not make it virtual as this would hinder performance.
  float getDistanceToObject(int i, int j);

protected:
  struct OccupancyMapCellData;

  virtual void iterateObstacleCells(std::priority_queue<OccupancyMapCellData>& q,
                                    std::vector<bool>& marked);
  virtual void iterateEmptyCells(std::priority_queue<OccupancyMapCellData>& q,
                                 std::vector<bool>& marked);
  virtual bool enqueue(int i, int j, int src_i, int src_j,
                       std::priority_queue<OccupancyMapCellData>& q);

  // Map dimensions (number of cells)
  int size_x_, size_y_;

  // The map occupancy data, stored as a grid
  std::vector<MapCellState> cells_;

  // The map distance data, stored as a grid
  std::vector<float> distances_lut_;

  CachedDistanceOccupancyMap cdm_;

  struct OccupancyMapCellData
  {
    OccupancyMapCellData() = delete;
    OccupancyMap* occ_map;
    OccupancyMapCellData(OccupancyMap* o_map) : occ_map(o_map) {}
    int i, j;
    int src_i, src_j;
    inline bool operator<(const OccupancyMapCellData& b) const
    {
      return occ_map->getDistanceToObject(i, j) > b.occ_map->getDistanceToObject(b.i, b.j);
    }
  };

private:
  inline void setDistanceToObject(int i, int j, float d);
  inline void updateNode(int i, int j, const OccupancyMapCellData& current_cell,
                         std::priority_queue<OccupancyMapCellData>& q, std::vector<bool>& marked);
  std::vector<int> map_vec_;
  std::vector<double> world_vec_;
};
}  // namespace amcl

#endif  // AMCL_MAP_OCCUPANCY_MAP_H

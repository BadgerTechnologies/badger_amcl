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
/**************************************************************************
 * Desc: Global map (grid-based)
 * Author: Andrew Howard
 **************************************************************************/

#ifndef AMCL_MAP_OCCUPANCY_MAP_H
#define AMCL_MAP_OCCUPANCY_MAP_H

#include <cmath>
#include <cstdint>
#include <memory>
#include <queue>
#include <vector>

#include "map/map.h"

namespace amcl
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

  std::vector<std::vector<double>> distances_;
  double resolution_;
  double max_dist_;
  int cell_radius_;
};

struct OccupancyMapCellData;

class OccupancyMap : public Map
{
public:
  OccupancyMap(double resolution);
  virtual ~OccupancyMap() {};
  // Convert from map index to world coords
  virtual void convertMapToWorld(const std::vector<int>& map_coords,
                                 std::vector<double>* world_coords);
  // Convert from world coords to map coords
  virtual void convertWorldToMap(const std::vector<double>& world_coords,
                                 std::vector<int>* map_coords);
  // Test to see if the given map coords lie within the absolute map bounds.
  virtual bool isValid(const std::vector<int>& coords);
  virtual std::vector<int> getSize();
  virtual void setSize(std::vector<int> size_vec);
  // Update the cspace distance values
  virtual void updateCSpace(double max_occ_dist);
  // Extract a single range reading from the map
  virtual double calcRange(double ox, double oy, double oa, double max_range);
  // Find the distance to nearest occupied cell
  virtual float getOccDist(int i, int j);
  // Compute the cell index for the given map coords.
  virtual unsigned int computeCellIndex(int i, int j);
  virtual double getMaxOccDist();
  virtual void initCells(int num);
  virtual MapCellState getCellState(int i, int j);
  virtual void setCellState(int index, MapCellState state);

protected:
  void setMapOccDist(int i, int j, float d);
  void iterateObstacleCells();
  void iterateEmptyCells();
  void updateNode(int i, int j, const OccupancyMapCellData& current_cell);
  bool enqueue(int i, int j, int src_i, int src_j);

  // Map dimensions (number of cells)
  int size_x_, size_y_;

  // The map occupancy data, stored as a grid
  std::vector<MapCellState> cells_;

  // The map distance data, stored as a grid
  std::vector<float> distances_;

  CachedDistanceOccupancyMap cdm_;
  std::priority_queue<OccupancyMapCellData> q_;
  std::vector<bool> marked_;
};

struct OccupancyMapCellData
{
  OccupancyMap* occ_map;
  OccupancyMapCellData(OccupancyMap* o_map) : occ_map(o_map) {}
  int i, j;
  int src_i, src_j;
  inline bool operator<(const OccupancyMapCellData& b) const
  {
    return occ_map->getOccDist(i, j) > b.occ_map->getOccDist(b.i, b.j);
  }
};
}  // namespace amcl

#endif  // AMCL_MAP_OCCUPANCY_MAP_H

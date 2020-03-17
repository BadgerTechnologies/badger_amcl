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
 *************************************************************************/

#include "map/occupancy_map.h"

#include <ros/console.h>
#include <stdlib.h>

#include <string>

using namespace amcl;

// Update the cspace distance values
void OccupancyMap::updateCSpace(double max_occ_dist)
{
  max_occ_dist_ = max_occ_dist;
  if(max_occ_dist_ == 0.0)
  {
    ROS_DEBUG("Failed to update cspace, max occ dist is 0");
    return;
  }

  ROS_INFO("Updating Occupancy Map CSpace");
  q_ = std::priority_queue<OccupancyMapCellData>();
  unsigned s = unsigned(size_x_) * size_y_;
  std::fill(marked_.begin(), marked_.end(), false);
  marked_.resize(s, false);
  distances_.resize(unsigned(size_x_) * size_y_);
  if ((cdm_.resolution_ != resolution_) || (cdm_.max_dist_ != max_occ_dist_))
  {
    cdm_ = CachedDistanceOccupancyMap(resolution_, max_occ_dist_);
  }
  iterateObstacleCells();
  iterateEmptyCells();
  cspace_created_ = true;
  ROS_INFO("Done updating Occupancy Map CSpace");
}

void OccupancyMap::iterateObstacleCells()
{
  // Enqueue all the obstacle cells
  OccupancyMapCellData cell = OccupancyMapCellData(this);
  for (int i = 0; i < size_x_; i++)
  {
    cell.src_i = cell.i = i;
    for (int j = 0; j < size_y_; j++)
    {
      if (getCellState(i, j) == MapCellState::CELL_OCCUPIED)
      {
        setMapOccDist(i, j, 0.0);
        cell.src_j = cell.j = j;
        marked_.at(computeCellIndex(i, j)) = true;
        q_.push(cell);
      }
      else
      {
        setMapOccDist(i, j, max_occ_dist_);
      }
    }
  }
}

void OccupancyMap::iterateEmptyCells()
{
  while (!q_.empty())
  {
    OccupancyMapCellData current_cell = q_.top();
    if (current_cell.i > 0)
    {
      updateNode(current_cell.i - 1, current_cell.j, current_cell);
    }
    if (current_cell.j > 0)
    {
      updateNode(current_cell.i, current_cell.j - 1, current_cell);
    }
    if ((int)current_cell.i < size_x_ - 1)
    {
      updateNode(current_cell.i + 1, current_cell.j, current_cell);
    }
    if ((int)current_cell.j < size_y_ - 1)
    {
      updateNode(current_cell.i, current_cell.j + 1, current_cell);
    }
    q_.pop();
  }
}

void OccupancyMap::updateNode(int i, int j, const OccupancyMapCellData& current_cell)
{
  unsigned int index = computeCellIndex(i, j);
  if (not marked_.at(index))
  {
    marked_.at(index) = enqueue(i, j, current_cell.src_i, current_cell.src_j);
  }
}

bool OccupancyMap::enqueue(int i, int j, int src_i, int src_j)
{
  int di = abs(i - src_i);
  int dj = abs(j - src_j);
  double distance = cdm_.distances_[di][dj];
  if (distance > cdm_.cell_radius_)
  {
    setMapOccDist(i, j, distance * resolution_);
    OccupancyMapCellData cell = OccupancyMapCellData(this);
    cell.i = i;
    cell.j = j;
    cell.src_i = src_i;
    cell.src_j = src_j;
    q_.push(cell);
    return true;
  }
  return false;
}

void OccupancyMap::setMapOccDist(int i, int j, float d)
{
  if (isValid({ i, j }))
  {
    distances_[computeCellIndex(i, j)] = d;
  }
}

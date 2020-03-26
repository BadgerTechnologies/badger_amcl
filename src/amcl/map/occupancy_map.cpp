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

#include <algorithm>
#include <cstdlib>

namespace amcl
{

OccupancyMap::OccupancyMap(double resolution) :
    Map(resolution),
    size_x_(0),
    size_y_(0),
    cdm_(resolution, 0.0)
{
  max_occ_dist_ = 0.0;
}

std::vector<int> OccupancyMap::getSize()
{
  return { size_x_, size_y_ };
}

void OccupancyMap::setSize(std::vector<int> size_vec)
{
  size_x_ = size_vec[0];
  size_y_ = size_vec[1];
}

double OccupancyMap::getMaxOccDist()
{
  return max_occ_dist_;
}

void OccupancyMap::initCells(int num)
{
  cells_.resize(num);
}

float OccupancyMap::getOccDist(int i, int j)
{
  if (isValid({ i, j }))
  {
    return distances_[computeCellIndex(i, j)];
  }
  return max_occ_dist_;
}

void OccupancyMap::convertMapToWorld(const std::vector<int>& map_coords,
                                     std::vector<double>* world_coords)
{
  std::vector<double> return_vals;
  int i = map_coords[0];
  int j = map_coords[1];
  (*world_coords)[0] = origin_.x + (i - size_x_ / 2) * resolution_;
  (*world_coords)[1] = origin_.y + (j - size_y_ / 2) * resolution_;
}

void OccupancyMap::convertWorldToMap(const std::vector<double>& world_coords,
                                     std::vector<int>* map_coords)
{
  std::vector<int> return_vals;
  double x = world_coords[0];
  double y = world_coords[1];
  (*map_coords)[0] = std::floor((x - origin_.x) / resolution_ + 0.5) + size_x_ / 2;
  (*map_coords)[1] = std::floor((y - origin_.y) / resolution_ + 0.5) + size_y_ / 2;
}

bool OccupancyMap::isValid(const std::vector<int>& coords)
{
  int i = coords[0];
  int j = coords[1];
  return (i >= 0) && (i < size_x_) && (j >= 0) && (j < size_y_);
}

unsigned int OccupancyMap::computeCellIndex(int i, int j)
{
  return i + j * unsigned(size_x_);
}

void OccupancyMap::setCellState(int index, MapCellState state)
{
  cells_[index] = state;
}

MapCellState OccupancyMap::getCellState(int i, int j)
{
  return cells_[computeCellIndex(i, j)];
}

CachedDistanceOccupancyMap::CachedDistanceOccupancyMap(double resolution, double max_dist)
    : resolution_(resolution), max_dist_(max_dist)
{
  cell_radius_ = static_cast<int>(std::floor(max_dist / resolution));
  distances_.resize(cell_radius_ + 2);
  for (int i = 0; i <= cell_radius_ + 1; i++)
  {
    distances_[i].resize(cell_radius_ + 2);
    for (int j = 0; j <= cell_radius_ + 1; j++)
    {
      distances_[i][j] = std::sqrt(i * i + j * j);
    }
  }
}

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
  int di = std::abs(i - src_i);
  int dj = std::abs(j - src_j);
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

// Extract a single range reading from the map.  Unknown cells and/or
// out-of-bound cells are treated as occupied, which makes it easy to
// use Stage bitmap files.
double OccupancyMap::calcRange(double ox, double oy, double oa, double max_range)
{
  // Bresenham raytracing
  std::vector<int> tmp_vec(2);
  int x0, x1, y0, y1;
  int x, y;
  int xstep, ystep;
  char steep;
  int tmp;
  int deltax, deltay, error, deltaerr;

  convertWorldToMap({ ox, oy }, &tmp_vec);
  x0 = tmp_vec[0];
  y0 = tmp_vec[1];
  convertWorldToMap({ ox + max_range * std::cos(oa), oy + max_range * sin(oa) }, &tmp_vec);
  x1 = tmp_vec[0];
  y1 = tmp_vec[1];

  if (std::abs(y1 - y0) > std::abs(x1 - x0))
    steep = 1;
  else
    steep = 0;

  if (steep)
  {
    tmp = x0;
    x0 = y0;
    y0 = tmp;

    tmp = x1;
    x1 = y1;
    y1 = tmp;
  }

  deltax = std::abs(x1 - x0);
  deltay = std::abs(y1 - y0);
  error = 0;
  deltaerr = deltay;

  x = x0;
  y = y0;

  if (x0 < x1)
    xstep = 1;
  else
    xstep = -1;
  if (y0 < y1)
    ystep = 1;
  else
    ystep = -1;

  if (steep)
  {
    if (!isValid({ y, x }) || cells_[computeCellIndex(y, x)] != MapCellState::CELL_FREE)
      return std::sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0)) * resolution_;
  }
  else
  {
    if (!isValid({ x, y }) || cells_[computeCellIndex(x, y)] != MapCellState::CELL_FREE)
      return std::sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0)) * resolution_;
  }

  while (x != (x1 + xstep * 1))
  {
    x += xstep;
    error += deltaerr;
    if (2 * error >= deltax)
    {
      y += ystep;
      error -= deltax;
    }

    if (steep)
    {
      if (!isValid({ y, x }) || cells_[computeCellIndex(y, x)] != MapCellState::CELL_FREE)
        return std::sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0)) * resolution_;
    }
    else
    {
      if (!isValid({ x, y }) || cells_[computeCellIndex(x, y)] != MapCellState::CELL_FREE)
        return std::sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0)) * resolution_;
    }
  }
  return max_range;
}

}  //namespace amcl

/*
 *  Player - One Hell of a Robot Server
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
 * Maintainter: Tyler Buchman (tyler_buchman@jabil.com)
**************************************************************************/

#include "map/occupancy_map.h"

using namespace amcl;

// Create a new map
OccupancyMap::OccupancyMap()
{
  // Assume we start at (0, 0)
  origin_x_ = 0;
  origin_y_ = 0;

  // Make the size odd
  size_x_ = 0;
  size_y_ = 0;

  max_occ_dist_ = 0;

  // Allocate storage for main map
  cdm_ = nullptr;
}

std::vector<double>
OccupancyMap::getOrigin()
{
  return {origin_x_, origin_y_};
}

void
OccupancyMap::setOrigin(std::vector<double> origin)
{
  origin_x_ = origin[0];
  origin_y_ = origin[1];
}

std::vector<int>
OccupancyMap::getSize()
{
  return {size_x_, size_y_};
}

void
OccupancyMap::setSize(std::vector<int> size_vec)
{
  size_x_ = size_vec[0];
  size_y_ = size_vec[1];
}

double
OccupancyMap::getMaxOccDist()
{
  return max_occ_dist_;
}

void
OccupancyMap::initCells(int num)
{
  cells_.resize(num);
}

void
OccupancyMap::setCellOccState(int index, int8_t state)
{
  cells_[index].occ_state = state;
}

float
OccupancyMap::getOccDist(int i, int j)
{
  if(isValid({i, j}))
  {
    return distances_[computeCellIndex(i, j)];
  }
  return max_occ_dist_;
}

void
OccupancyMap::convertMapToWorld(const std::vector<int> &map_coords,
                                std::vector<double> *world_coords)
{
  std::vector<double> return_vals;
  int i = map_coords[0];
  int j = map_coords[1];
  (*world_coords)[0] = origin_x_ + (i - size_x_ / 2) * scale_;
  (*world_coords)[1] = origin_y_ + (j - size_y_ / 2) * scale_;
}

void
OccupancyMap::convertWorldToMap(const std::vector<double> &world_coords,
                                std::vector<int> *map_coords)
{
  std::vector<int> return_vals;
  double x = world_coords[0];
  double y = world_coords[1];
  (*map_coords)[0] = floor((x - origin_x_) / scale_ + 0.5) + size_x_ / 2;
  (*map_coords)[1] = floor((y - origin_y_) / scale_ + 0.5) + size_y_ / 2;
}

bool
OccupancyMap::isValid(std::vector<int> coords)
{
  int i = coords[0];
  int j = coords[1];
  return (i >= 0) && (i < size_x_) && (j >= 0) && (j < size_y_);
}

unsigned int
OccupancyMap::computeCellIndex(int i, int j)
{
  return i + j * unsigned(size_x_);
}

int8_t
OccupancyMap::getOccState(int i, int j)
{
  return cells_[computeCellIndex(i, j)].occ_state;
}

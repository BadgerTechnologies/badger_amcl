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
 * Date: 6 Feb 2003
 * CVS: $Id: map.c 1713 2003-08-23 04:03:43Z inspectorg $
**************************************************************************/

#include "map.h"
#include "occupancy_map.h"
#include <queue>
#include <stdint.h>

using namespace amcl;

// Create a new map
OccupancyMap::OccupancyMap()
{
  // Assume we start at (0, 0)
  origin_x = 0;
  origin_y = 0;

  // Make the size odd
  size_x = 0;
  size_y = 0;

  max_occ_dist = 0;

  // Allocate storage for main map
  cells = nullptr;
  distances = nullptr;

  cdm = nullptr;
}

// Destroy a map
OccupancyMap::~OccupancyMap()
{
  delete[] distances;
  delete[] cells;
}

std::vector<double>
OccupancyMap::getOrigin()
{
  return {origin_x, origin_y};
}

void
OccupancyMap::setOrigin(std::vector<double> _origin)
{
  origin_x = _origin[0];
  origin_y = _origin[1];
}

std::vector<int>
OccupancyMap::getSize()
{
  return {size_x, size_y};
}

void
OccupancyMap::setSize(std::vector<int> _size)
{
  size_x = _size[0];
  size_y = _size[1];
}

double
OccupancyMap::getMaxOccDist()
{
  return max_occ_dist;
}

map_cell_t*
OccupancyMap::getCells()
{
  return cells;
}

void
OccupancyMap::initCells(int num)
{
  if(cells)
      delete[] cells;
  cells = new map_cell_t[num];
}

void
OccupancyMap::setCellOccState(int index, int8_t state)
{
  cells[index].occ_state = state;
}

float
OccupancyMap::occDist(int i, int j)
{
  if(isValid({i, j}))
  {
    return distances[computeCellIndex(i, j)];
  }
  return max_occ_dist;

}

std::vector<double>
OccupancyMap::convertMapToWorld(std::vector<int> map_coords)
{
  std::vector<double> return_vals;
  int i = map_coords[0];
  int j = map_coords[1];
  return_vals.push_back(origin_x + (i - size_x / 2) * scale);
  return_vals.push_back(origin_y + (j - size_y / 2) * scale);
  return return_vals;
}

std::vector<int>
OccupancyMap::convertWorldToMap(std::vector<double> world_coords)
{
  std::vector<int> return_vals;
  double x = world_coords[0];
  double y = world_coords[1];
  return_vals.push_back(floor((x - origin_x) / scale + 0.5) + size_x / 2);
  return_vals.push_back(floor((y - origin_y) / scale + 0.5) + size_y / 2);
  return return_vals;
}

bool
OccupancyMap::isValid(std::vector<int> coords)
{
  int i = coords[0];
  int j = coords[1];
  return (i >= 0) && (i < size_x) && (j >= 0) && (j < size_y);
}

unsigned int
OccupancyMap::computeCellIndex(int i, int j)
{
  return i + j * unsigned(size_x);
}

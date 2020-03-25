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
 * Desc: Range routines
 * Author: Andrew Howard
 *************************************************************************/

#include "map/occupancy_map.h"

#include <cstdlib>

using namespace amcl;

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

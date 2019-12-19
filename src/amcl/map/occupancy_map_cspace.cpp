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

#include <ros/console.h>
#include <stdlib.h>

#include <string>

using namespace amcl;

bool
OccupancyMap::enqueue(int i, int j, int src_i, int src_j,
                      std::priority_queue<CellData>& Q)
{
  int di = abs(i - src_i);
  int dj = abs(j - src_j);
  double distance = cdm_->distances_[di][dj];

  if(distance > cdm_->cell_radius_)
    return false;

  setMapOccDist(i, j, distance * scale_);

  CellData cell = CellData(this);
  cell.i_ = i;
  cell.j_ = j;
  cell.src_i_ = src_i;
  cell.src_j_ = src_j;

  Q.push(cell);

  return true;
}

// Update the cspace distance values
void
OccupancyMap::updateCSpace(double max_occ_dist)
{
  ROS_INFO("Updating Occupancy Map CSpace");
  std::priority_queue<CellData> Q;

  distances_.resize(unsigned(size_x_)*size_y_);

  std::vector<bool> marked(unsigned(size_x_) * size_y_, false);

  max_occ_dist_ = max_occ_dist;

  if(!cdm_ || (cdm_->scale_ != scale_) || (cdm_->max_dist_ != max_occ_dist_))
  {
    cdm_ = std::unique_ptr<CachedDistanceMap>(new CachedDistanceMap(scale_, max_occ_dist_));
  }

  // Enqueue all the obstacle cells
  CellData cell = CellData(this);
  for(int i=0; i<size_x_; i++)
  {
    cell.src_i_ = cell.i_ = i;
    for(int j=0; j<size_y_; j++)
    {
      if(getOccState(i, j) == +1)
      {
        setMapOccDist(i, j, 0.0);
	    cell.src_j_ = cell.j_ = j;
	    marked[computeCellIndex(i, j)] = true;
	    Q.push(cell);
      }
      else
      {
	    setMapOccDist(i, j, max_occ_dist);
      }
    }
  }

  while(!Q.empty())
  {
    CellData current_cell = Q.top();
    if(current_cell.i_ > 0)
    {
      int i = current_cell.i_-1, j = current_cell.j_;
      unsigned int index = computeCellIndex(i, j);
      if(not marked[index])
        marked[index] = enqueue(i, j, current_cell.src_i_, current_cell.src_j_, Q);
    }
    if(current_cell.j_ > 0)
    {
      int i = current_cell.i_, j = current_cell.j_-1;
      unsigned int index = computeCellIndex(i, j);
      if(not marked[index])
        marked[index] = enqueue(i, j, current_cell.src_i_, current_cell.src_j_, Q);
    }
    if((int)current_cell.i_ < size_x_ - 1)
    {
      int i = current_cell.i_+1, j = current_cell.j_;
      unsigned int index = computeCellIndex(i, j);
      if(not marked[index])
        marked[index] = enqueue(i, j, current_cell.src_i_, current_cell.src_j_, Q);
    }
    if((int)current_cell.j_ < size_y_ - 1)
    {
      int i = current_cell.i_, j = current_cell.j_+1;
      unsigned int index = computeCellIndex(i, j);
      if(not marked[index])
        marked[index] = enqueue(i, j, current_cell.src_i_, current_cell.src_j_, Q);
    }
    Q.pop();
  }
  cspace_created_ = true;
  ROS_INFO("Done updating Occupancy Map CSpace");
}

void
OccupancyMap::setMapOccDist(int i, int j, float d)
{
  if (isValid({i, j}))
  {
    distances_[computeCellIndex(i, j)] = d;
  }
}

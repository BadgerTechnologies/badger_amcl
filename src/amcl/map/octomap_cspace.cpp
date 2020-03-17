/*  Copyright (C) 2020 Badger Technologies, LLC
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
 *************************************************************************/

#include "map/octomap.h"

#include <boost/functional/hash.hpp>
#include <octomap/OcTreeKey.h>
#include <octomap/OcTreeDataNode.h>
#include <octomap/OcTreeNode.h>
#include <ros/console.h>
#include <stdlib.h>

#include <algorithm>
#include <cmath>

using namespace amcl;

// This version also updates the max_occ_dist_ variable and
// calls the base non-parameter updateCSpace
void OctoMap::updateMaxOccDist(double max_occ_dist)
{
  max_occ_dist_ = max_occ_dist;
}

// Creates the distances lookup object populated with the distance from
// each voxel to the nearest object in the static map
void OctoMap::updateCSpace()
{
  if (max_occ_dist_ == 0.0)
  {
    ROS_DEBUG("Failed to update cspace, max occ dist is 0");
    return;
  }

  ROS_INFO("Updating OctoMap CSpace");
  q_ = std::priority_queue<OctoMapCellData>();
  marked_.clear();
  marked_.setResolution(resolution_);
  distances_.clear();
  if ((cdm_.resolution_ != resolution_) || (std::fabs(cdm_.max_dist_ - max_occ_dist_) > EPSILON))
  {
    cdm_ = CachedDistanceOctoMap(resolution_, max_occ_dist_);
  }
  iterateObstacleCells();
  iterateEmptyCells();
  distances_end_ = distances_.end();
  cspace_created_ = true;
  ROS_INFO("Done updating OctoMap CSpace");
}

void OctoMap::iterateObstacleCells()
{
  // Enqueue all the obstacle cells
  OctoMapCellData cell = OctoMapCellData(*this);
  std::vector<double> world_coords(3);
  std::vector<int> map_coords(3);

  for (octomap::OcTree::leaf_iterator it = octree_->begin_leafs(), end = octree_->end_leafs(); it != end; ++it)
  {
    if (octree_->isNodeOccupied(*it))
    {
      int i, j, k;
      world_coords[0] = it.getX();
      world_coords[1] = it.getY();
      world_coords[2] = it.getZ();
      convertWorldToMap(world_coords, &map_coords);
      i = map_coords[0];
      j = map_coords[1];
      k = map_coords[2];
      setOccDist(i, j, k, 0.0);
      cell.src_i = cell.i = i;
      cell.src_j = cell.j = j;
      cell.src_k = cell.k = k;
      marked_.updateNode(i, j, k, true);
      q_.push(cell);
    }
    else
    {
      octomap::OcTreeKey key = it.getIndexKey();
      int i = key[0], j = key[1], k = key[2];
      setOccDist(i, j, k, max_occ_dist_);
    }
  }
}

void OctoMap::iterateEmptyCells()
{
  while (!q_.empty())
  {
    OctoMapCellData current_cell = q_.top();
    if (current_cell.i > cropped_min_cells_[0])
    {
      updateNode(current_cell.i - 1, current_cell.j, current_cell.k, current_cell);
    }
    if (current_cell.j > cropped_min_cells_[1])
    {
      updateNode(current_cell.i, current_cell.j - 1, current_cell.k, current_cell);
    }
    if (current_cell.k > cropped_min_cells_[2])
    {
      updateNode(current_cell.i, current_cell.j, current_cell.k - 1, current_cell);
    }
    if (current_cell.i < cropped_max_cells_[0] - 1)
    {
      updateNode(current_cell.i + 1, current_cell.j, current_cell.k, current_cell);
    }
    if (current_cell.j < cropped_max_cells_[1] - 1)
    {
      updateNode(current_cell.i, current_cell.j + 1, current_cell.k, current_cell);
    }
    if (current_cell.k < cropped_max_cells_[2] - 1)
    {
      updateNode(current_cell.i, current_cell.j, current_cell.k + 1, current_cell);
    }
    q_.pop();
  }
}

void OctoMap::updateNode(int i, int j, int k, const OctoMapCellData& current_cell)
{
  double occThresh = octree_->getOccupancyThres();
  octomap::OcTreeKey key(i, j, k);
  octomap::OcTreeNode* node = marked_.search(key);
  if (node == nullptr or not node->getOccupancy() > occThresh)
  {
    bool enqueued = enqueue(i, j, k, current_cell.src_i, current_cell.src_j, current_cell.src_k);
    marked_.updateNode(key, enqueued);
  }
}

// Helper function for updateCSpace
// Adds the voxel to the queue if the voxel is close enough to an object
bool OctoMap::enqueue(int i, int j, int k, int src_i, int src_j, int src_k)
{
  int di = abs(i - src_i);
  int dj = abs(j - src_j);
  int dk = abs(k - src_k);
  double distance = cdm_.distances_[di][dj][dk];

  if (distance <= cdm_.cell_radius_)
  {
    setOccDist(i, j, k, distance * resolution_);
    OctoMapCellData cell = OctoMapCellData(*this);
    cell.i = i;
    cell.j = j;
    cell.k = k;
    cell.src_i = src_i;
    cell.src_j = src_j;
    cell.src_k = src_k;
    q_.push(cell);
    return true;
  }
  return false;
}

// Helper function for updateCSpace
// Sets the distance from the voxel to the nearest object in the static map
void OctoMap::setOccDist(int i, int j, int k, double d)
{
  size_t hash = makeHash(i, j, k);
  distances_.insert_or_assign(hash, d);
}

// returns the distance from the 3d voxel to the nearest object in the static map
double OctoMap::getOccDist(int i, int j, int k)
{
  size_t hash = makeHash(i, j, k);
  hashmap_iterator_ = distances_.find(hash);
  if(hashmap_iterator_ != distances_end_)
  {
    return hashmap_iterator_->second;
  }
  return max_occ_dist_;
}

size_t OctoMap::makeHash(int i, int j, int k)
{
  std::size_t hash(0);
  boost::hash_combine(hash, i);
  boost::hash_combine(hash, j);
  boost::hash_combine(hash, k);
  return hash;
}

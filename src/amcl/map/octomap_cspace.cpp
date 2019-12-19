/*  This library is free software; you can redistribute it and/or
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
 * Author: Tyler Buchman (tyler_buchman@jabil.com)
**************************************************************************/

#include "map/octomap.h"

#include <math.h>
#include <octomap/OcTreeKey.h>
#include <octomap/OcTreeDataNode.h>
#include <octomap/OcTreeNode.h>
#include <ros/console.h>
#include <stdlib.h>

#include <algorithm>

using namespace amcl;

// This version also updates the max_occ_dist_ variable and
// calls the base non-parameter updateCSpace
void
OctoMap::updateMaxOccDist(double max_occ_dist)
{
  this->max_occ_dist_ = max_occ_dist;
}

// Creates the distances lookup object populated with the distance from
// each voxel to the nearest object in the static map
void
OctoMap::updateCSpace()
{
  if(max_occ_dist_ == 0.0)
  {
    ROS_DEBUG("Failed to update cspace, max occ dist is 0");
    return;
  }

  ROS_INFO("Updating OctoMap CSpace");
  std::priority_queue<CellData> Q;
  octomap::OcTree* marked = new octomap::OcTree(scale_);

  distances_ = std::unique_ptr<octomap::OcTreeDistance>(new octomap::OcTreeDistance(scale_, max_occ_dist_));

  if(!cdm_ || (cdm_->scale_ != scale_)
     || (std::fabs(cdm_->max_dist_ - max_occ_dist_) > EPSILON_DOUBLE))
  {
    cdm_ = std::unique_ptr<CachedDistanceMap>(new CachedDistanceMap(scale_, max_occ_dist_));
  }

  // Enqueue all the obstacle cells
  CellData cell = CellData(*this);
  std::vector<double> world_coords(3);
  std::vector<int> map_coords(3);

  for(octomap::OcTree::leaf_iterator it = octree_->begin_leafs(), end=octree_->end_leafs();
      it != end; ++it)
  {
    if(octree_->isNodeOccupied(*it))
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
      cell.src_i_ = cell.i_ = i;
      cell.src_j_ = cell.j_ = j;
      cell.src_k_ = cell.k_ = k;
      marked->updateNode(i, j, k, true);
      Q.push(cell);
    }
    else
    {
      octomap::OcTreeKey key = it.getIndexKey();
      int i = key[0], j = key[1], k = key[2];
      setOccDist(i, j, k, max_occ_dist_);
    }
  }

  double occThresh = octree_->getOccupancyThres();

  while(!Q.empty())
  {
    CellData current_cell = Q.top();
    if(current_cell.i_ > cropped_min_cells_[0])
    {
      int i = current_cell.i_-1, j = current_cell.j_, k = current_cell.k_;
      octomap::OcTreeKey key(i, j, k);
      octomap::OcTreeNode* node = marked->search(key);
      if(node == nullptr or not node->getOccupancy() > occThresh)
        marked->updateNode(key, enqueue(i, j, k, current_cell.src_i_, current_cell.src_j_, current_cell.src_k_, Q));
    }
    if(current_cell.j_ > cropped_min_cells_[1])
    {
      int i = current_cell.i_, j = current_cell.j_-1, k = current_cell.k_;
      octomap::OcTreeKey key(i, j, k);
      octomap::OcTreeNode* node = marked->search(key);
      if(node == nullptr or not node->getOccupancy() > occThresh)
        marked->updateNode(key, enqueue(i, j, k, current_cell.src_i_, current_cell.src_j_, current_cell.src_k_, Q));
    }
    if(current_cell.k_ > cropped_min_cells_[2])
    {
      int i = current_cell.i_, j = current_cell.j_, k = current_cell.k_-1;
      octomap::OcTreeKey key(i, j, k);
      octomap::OcTreeNode* node = marked->search(key);
      if(node == nullptr or not node->getOccupancy() > occThresh)
        marked->updateNode(key, enqueue(i, j, k, current_cell.src_i_, current_cell.src_j_, current_cell.src_k_, Q));
    }
    if((int)current_cell.i_ < cropped_max_cells_[0] - 1)
    {
      int i = current_cell.i_+1, j = current_cell.j_, k = current_cell.k_;
      octomap::OcTreeKey key(i, j, k);
      octomap::OcTreeNode* node = marked->search(key);
      if(node == nullptr or not node->getOccupancy() > occThresh)
        marked->updateNode(key, enqueue(i, j, k, current_cell.src_i_, current_cell.src_j_, current_cell.src_k_, Q));
    }
    if((int)current_cell.j_ < cropped_max_cells_[1] - 1)
    {
      int i = current_cell.i_, j = current_cell.j_+1, k = current_cell.k_;
      octomap::OcTreeKey key(i, j, k);
      octomap::OcTreeNode* node = marked->search(key);
      if(node == nullptr or not node->getOccupancy() > occThresh)
        marked->updateNode(key, enqueue(i, j, k, current_cell.src_i_, current_cell.src_j_, current_cell.src_k_, Q));
    }
    if((int)current_cell.k_ < cropped_max_cells_[2] - 1)
    {
      int i = current_cell.i_, j = current_cell.j_, k = current_cell.k_+1;
      octomap::OcTreeKey key(i, j, k);
      octomap::OcTreeNode* node = marked->search(key);
      if(node == nullptr or not node->getOccupancy() > occThresh)
        marked->updateNode(key, enqueue(i, j, k, current_cell.src_i_, current_cell.src_j_, current_cell.src_k_, Q));
    }
    Q.pop();
  }
  cspace_created_ = true;
  ROS_INFO("Done updating OctoMap CSpace");
}

// Helper function for updateCSpace
// Adds the voxel to the queue if the voxel is close enough to an object
bool
OctoMap::enqueue(int i, int j, int k, int src_i, int src_j, int src_k,
                 std::priority_queue<CellData>& Q)
{
  int di = abs(i - src_i);
  int dj = abs(j - src_j);
  int dk = abs(k - src_k);
  double distance = cdm_->distances_[di][dj][dk];

  if(distance > cdm_->cell_radius_)
    return false;

  setOccDist(i, j, k, distance * scale_);

  CellData cell = CellData(*this);
  cell.i_ = i;
  cell.j_ = j;
  cell.k_ = k;
  cell.src_i_ = src_i;
  cell.src_j_ = src_j;
  cell.src_k_ = src_k;

  Q.push(cell);

  return true;
}

// Helper function for updateCSpace
// Sets the distance from the voxel to the nearest object in the static map
void
OctoMap::setOccDist(int i, int j, int k, double d)
{
  int index = floor(d*100);
  octomap::OcTreeKey key(i, j, k);
  distances_->setDistanceForKey(key, d);
}

// returns the distance from the 3d voxel to the nearest object in the static map
double
OctoMap::getOccDist(int i, int j, int k)
{
  octomap::OcTreeKey key(i, j, k);
  octomap::OcTreeDataNode<double>* node = distances_->search(key);
  if(node and node->getValue() < max_occ_dist_)
  {
    return node->getValue();
  }
  return 1*max_occ_dist_;
}

// returns the distance from the 2d pose to the nearest object in a 2.5d view
double
OctoMap::getOccDist(int i, int j)
{
  double distance = max_occ_dist_;
  for(int k = 0; k < lidar_height_ / scale_; k++)
  {
    distance = std::min(distance, getOccDist(i, j, k));
  }
  return distance;
}

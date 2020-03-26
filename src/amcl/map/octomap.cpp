/*
 *  Copyright (C) 2020 Badger Technologies, LLC
 *
 *   This library is free software; you can redistribute it and/or
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

#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace amcl
{

OctoMap::OctoMap(double resolution, bool wait_for_occupancy_map)
    : Map(resolution),
      wait_for_occupancy_map_(wait_for_occupancy_map_),
      cdm_(resolution, 0.0),
      marked_(resolution)
{
  full_cells_ = std::vector<int>(3);
  cropped_min_cells_ = std::vector<int>(3);
  cropped_max_cells_ = std::vector<int>(3);
  map_min_bounds_ = std::vector<double>(2);
  map_max_bounds_ = std::vector<double>(2);
  max_occ_dist_ = 0.0;
  octree_ = std::make_shared<octomap::OcTree>(resolution_);
}

// initialize octomap from octree
void OctoMap::initFromOctree(std::shared_ptr<octomap::OcTree> octree)
{
  octree_ = octree;
  // set size
  double x_meters, y_meters, z_meters;
  octree_->getMetricSize(x_meters, y_meters, z_meters);
  full_cells_[0] = (int)std::ceil(x_meters / resolution_);
  full_cells_[1] = (int)std::ceil(y_meters / resolution_);
  full_cells_[2] = (int)std::ceil(z_meters / resolution_);
  double min_x, min_y, min_z, max_x, max_y, max_z;
  octree_->getMetricMin(min_x, min_y, min_z);
  octree_->getMetricMax(max_x, max_y, max_z);
  setOrigin(pcl::PointXYZ(min_x, min_y, min_z));
  // crop values here if required
  convertWorldToMap({ min_x, min_y, min_z }, &cropped_min_cells_);
  convertWorldToMap({ max_x, max_y, max_z }, &cropped_max_cells_);
}

// returns vector of map size in voxels
// each voxel represents map_size_in_meters / resolution
std::vector<int> OctoMap::getSize()
{
  return full_cells_;
}

void OctoMap::getMinMaxCells(std::vector<int>* min_cells, std::vector<int>* max_cells)
{
  (*min_cells) = cropped_min_cells_;
  (*max_cells) = cropped_max_cells_;
}

// converts map voxel coordinates to global coordinates in meters
void OctoMap::convertMapToWorld(const std::vector<int>& map_coords, std::vector<double>* world_coords)
{
  std::vector<double> return_vals;
  int i = map_coords[0];
  int j = map_coords[1];
  (*world_coords)[0] = origin_.x + i * resolution_;
  (*world_coords)[1] = origin_.y + j * resolution_;
  if (map_coords.size() > 2)
  {
    int k = map_coords[2];
    (*world_coords)[2] = origin_.z + k * resolution_;
  }
}

// converts global coordinates in meters to map voxel coordinates
void OctoMap::convertWorldToMap(const std::vector<double>& world_coords, std::vector<int>* map_coords)
{
  double x = world_coords[0];
  double y = world_coords[1];
  (*map_coords)[0] = std::floor((x - origin_.x) / resolution_ + 0.5);
  (*map_coords)[1] = std::floor((y - origin_.y) / resolution_ + 0.5);
  if (world_coords.size() > 2)
  {
    double z = world_coords[2];
    (*map_coords)[2] = std::floor((z - origin_.z) / resolution_ + 0.5);
  }
}

// returns true if all coordinates are within the represented map
bool OctoMap::isValid(const std::vector<int>& coords)
{
  int i = coords[0];
  int j = coords[1];
  if ((i < cropped_min_cells_[0]) || (i >= cropped_max_cells_[0]) || (j < cropped_min_cells_[1]) ||
      (j >= cropped_max_cells_[1]))
    return false;
  if (coords.size() == 2)
    return true;
  else
  {
    int k = coords[2];
    return (k >= cropped_min_cells_[2]) && (k < cropped_max_cells_[2]);
  }
}

double OctoMap::getMaxOccDist()
{
  return max_occ_dist_;
}

void OctoMap::setMapBounds(const std::vector<double>& map_min,
                           const std::vector<double>& map_max)
{
  std::vector<int> cells_min(map_min.size()), cells_max(map_max.size());
  std::vector<double> map_min_local(map_min), map_max_local(map_max);
  // add a buffer around map bounds to ensure representation
  // of objects at extreme map values
  for (int i = 0; i < map_min_local.size(); i++)
  {
    map_min_local[i] -= max_occ_dist_;
    map_max_local[i] += max_occ_dist_;
  }
  convertWorldToMap(map_min_local, &cells_min);
  convertWorldToMap(map_max_local, &cells_max);
  for (int i = 0; i < cells_min.size(); i++)
  {
    cropped_min_cells_[i] = std::max(cropped_min_cells_[i], cells_min[i]);
    cropped_max_cells_[i] = std::min(cropped_max_cells_[i], cells_max[i]);
  }
  updateCSpace();
}

CachedDistanceOctoMap::CachedDistanceOctoMap(double resolution, double max_dist)
    : resolution_(resolution), max_dist_(max_dist)
{
  cell_radius_ = (int)std::floor(max_dist / resolution);
  distances_.resize(cell_radius_ + 2);
  for (int i = 0; i <= cell_radius_ + 1; i++)
  {
    distances_[i].resize(cell_radius_ + 2);
    for (int j = 0; j <= cell_radius_ + 1; j++)
    {
      distances_[i][j].resize(cell_radius_ + 2);
      for (int k = 0; k <= cell_radius_ + 1; k++)
      {
        distances_[i][j][k] = std::sqrt(i * i + j * j + k * k);
      }
    }
  }
}

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

  for (octomap::OcTree::leaf_iterator it = octree_->begin_leafs(),
          end = octree_->end_leafs(); it != end; ++it)
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
  int di = std::abs(i - src_i);
  int dj = std::abs(j - src_j);
  int dk = std::abs(k - src_k);
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
  std::size_t hash = makeHash(i, j, k);
  distances_.insert_or_assign(hash, d);
}

// returns the distance from the 3d voxel to the nearest object in the static map
double OctoMap::getOccDist(int i, int j, int k)
{
  std::size_t hash = makeHash(i, j, k);
  hashmap_iterator_ = distances_.find(hash);
  if(hashmap_iterator_ != distances_end_)
  {
    return hashmap_iterator_->second;
  }
  return max_occ_dist_;
}

std::size_t OctoMap::makeHash(int i, int j, int k)
{
  std::size_t hash(0);
  boost::hash_combine(hash, i);
  boost::hash_combine(hash, j);
  boost::hash_combine(hash, k);
  return hash;
}

}  // namespace amcl

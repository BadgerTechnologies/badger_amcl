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

#include "map/octomap.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>

#include <boost/functional/hash.hpp>
#include <octomap/OcTreeKey.h>
#include <octomap/OcTreeDataNode.h>
#include <octomap/OcTreeNode.h>
#include <ros/console.h>

namespace badger_amcl
{

OctoMap::OctoMap(double resolution)
    : Map(resolution),
      cdm_(resolution, 0.0)
{
  cropped_min_cells_ = std::vector<int>(3);
  cropped_max_cells_ = std::vector<int>(3);
  map_min_bounds_ = std::vector<double>(2);
  map_max_bounds_ = std::vector<double>(2);
  max_occ_dist_ = 0.0;
  octree_ = std::make_shared<octomap::OcTree>(resolution_);
  hash_function_ptr_ = std::bind(&OctoMap::makeHash, this, std::placeholders::_1);
  keys_equal_function_ptr_ = std::bind(&OctoMap::keysEqual, this, std::placeholders::_1, std::placeholders::_2);
  key_ = {0, 0, 0};
}

// initialize octomap from octree
void OctoMap::initFromOctree(std::shared_ptr<octomap::OcTree> octree)
{
  octree_ = octree;
  // set size
  double x_meters, y_meters, z_meters;
  octree_->getMetricSize(x_meters, y_meters, z_meters);
  double min_x, min_y, min_z, max_x, max_y, max_z;
  octree_->getMetricMin(min_x, min_y, min_z);
  octree_->getMetricMax(max_x, max_y, max_z);
  // crop values here if required
  convertWorldToMap({ min_x, min_y, min_z }, &cropped_min_cells_);
  convertWorldToMap({ max_x, max_y, max_z }, &cropped_max_cells_);
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
  (*world_coords)[0] = i * resolution_;
  (*world_coords)[1] = j * resolution_;
  if (map_coords.size() > 2)
  {
    int k = map_coords[2];
    (*world_coords)[2] = k * resolution_;
  }
}

// converts global coordinates in meters to map voxel coordinates
void OctoMap::convertWorldToMap(const std::vector<double>& world_coords, std::vector<int>* map_coords)
{
  double x = world_coords[0];
  double y = world_coords[1];
  (*map_coords)[0] = std::floor(x / resolution_ + 0.5);
  (*map_coords)[1] = std::floor(y / resolution_ + 0.5);
  if (world_coords.size() > 2)
  {
    double z = world_coords[2];
    (*map_coords)[2] = std::floor(z / resolution_ + 0.5);
  }
}

// returns true if all coordinates are within the represented map
bool OctoMap::isPoseValid(const std::vector<int>& coords)
{
  int i = coords[0];
  int j = coords[1];
  if ((i < cropped_min_cells_[0]) || (i >= cropped_max_cells_[0]) || (j < cropped_min_cells_[1]) ||
      (j >= cropped_max_cells_[1]))
    return false;
  return true;
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
  cell_radius_ = static_cast<int>(std::floor(max_dist / resolution));
  cached_distances_.resize(cell_radius_ + 2);
  for (int i = 0; i <= cell_radius_ + 1; i++)
  {
    cached_distances_[i].resize(cell_radius_ + 2);
    for (int j = 0; j <= cell_radius_ + 1; j++)
    {
      cached_distances_[i][j].resize(cell_radius_ + 2);
      for (int k = 0; k <= cell_radius_ + 1; k++)
      {
        cached_distances_[i][j][k] = std::sqrt(i * i + j * j + k * k);
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
  CellDataQueue q = CellDataQueue();
  int bucket_count = std::ceil(max_occ_dist_ / resolution_ * octree_->calcNumNodes());
  distances_ = HashMapDouble(bucket_count, hash_function_ptr_, keys_equal_function_ptr_);
  HashMapBool marked(bucket_count, hash_function_ptr_, keys_equal_function_ptr_);
  distances_.clear();
  if ((cdm_.resolution_ != resolution_) || (std::fabs(cdm_.max_dist_ - max_occ_dist_) > EPSILON))
  {
    cdm_ = CachedDistanceOctoMap(resolution_, max_occ_dist_);
  }
  ROS_INFO("Iterating obstacle cells");
  iterateObstacleCells(q, marked);
  ROS_INFO("Iterating empty cells");
  iterateEmptyCells(q, marked);
  cspace_created_ = true;
  ROS_INFO("Done updating OctoMap CSpace");
  ROS_INFO("Marked bucket count: %lu, marked max bucket count: %lu", marked.bucket_count(), marked.max_bucket_count());
  ROS_INFO("Distances bucket count; %lu, distances max bucket count: %lu", distances_.bucket_count(), distances_.max_bucket_count());
}

void OctoMap::iterateObstacleCells(CellDataQueue& q, HashMapBool& marked)
{
  // Enqueue all the obstacle cells
  OctoMapCellData cell = OctoMapCellData(*this);
  std::vector<double> world_coords(3);
  std::vector<int> map_coords(3);

  octree_->expand();
  int count = 0;
  for (octomap::OcTree::leaf_iterator it = octree_->begin_leafs(),
          end = octree_->end_leafs(); it != end; ++it)
  {
    if (octree_->isNodeOccupied(*it))
    {
      count++;
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
      key_[0] = i;
      key_[1] = j;
      key_[2] = k;
      marked.insert_or_assign(key_, true);
      if(!marked.find(key_)->second)
        ROS_INFO("boolean not working for key %d, %d, %d", key_[0], key_[1], key_[2]);
      q.push(cell);
    }
  }
  ROS_INFO("num_leaf_cells: %lu, num obstacle cells: %d", octree_->getNumLeafNodes(), count);
}

void OctoMap::iterateEmptyCells(CellDataQueue& q, HashMapBool& marked)
{
  int count = 0;
  while (!q.empty())
  {
    count += 1;
    count %= 1000000;
    if(count == 999)
      ROS_INFO("queue size: %lu", q.size());
    OctoMapCellData current_cell = q.top();
    if (current_cell.i > cropped_min_cells_[0])
    {
      updateNode(current_cell.i - 1, current_cell.j, current_cell.k, current_cell, q, marked);
    }
    if (current_cell.j > cropped_min_cells_[1])
    {
      updateNode(current_cell.i, current_cell.j - 1, current_cell.k, current_cell, q, marked);
    }
    if (current_cell.k > cropped_min_cells_[2])
    {
      updateNode(current_cell.i, current_cell.j, current_cell.k - 1, current_cell, q, marked);
    }
    if (current_cell.i < cropped_max_cells_[0] - 1)
    {
      updateNode(current_cell.i + 1, current_cell.j, current_cell.k, current_cell, q, marked);
    }
    if (current_cell.j < cropped_max_cells_[1] - 1)
    {
      updateNode(current_cell.i, current_cell.j + 1, current_cell.k, current_cell, q, marked);
    }
    if (current_cell.k < cropped_max_cells_[2] - 1)
    {
      updateNode(current_cell.i, current_cell.j, current_cell.k + 1, current_cell, q, marked);
    }
    q.pop();
  }
}

void OctoMap::updateNode(int i, int j, int k, const OctoMapCellData& current_cell,
                         CellDataQueue& q, HashMapBool& marked)
{
  double occThresh = octree_->getOccupancyThres();
  key_[0] = i;
  key_[1] = j;
  key_[2] = k;
  HashMapBool::iterator node = marked.find(key_);
  if (node == marked.end() or node->second == false)
  {
    bool enqueued = enqueue(i, j, k, current_cell.src_i, current_cell.src_j,
                            current_cell.src_k, q);
    marked.insert_or_assign(key_, enqueued);
  }
}

// Helper function for updateCSpace
// Adds the voxel to the queue if the voxel is close enough to an object
bool OctoMap::enqueue(int i, int j, int k, int src_i, int src_j, int src_k,
                      CellDataQueue& q)
{
  int di = std::abs(i - src_i);
  int dj = std::abs(j - src_j);
  int dk = std::abs(k - src_k);
  double distance = cdm_.cached_distances_[di][dj][dk];

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
    q.push(cell);
    return true;
  }
  return false;
}

// Helper function for updateCSpace
// Sets the distance from the voxel to the nearest object in the static map
void OctoMap::setOccDist(int i, int j, int k, double d)
{
  key_[0] = i;
  key_[1] = j;
  key_[2] = k;
  distances_.insert_or_assign(key_, d);
}

// returns the distance from the 3d voxel to the nearest object in the static map
double OctoMap::getOccDist(int i, int j, int k)
{
  key_[0] = i;
  key_[1] = j;
  key_[2] = k;
  HashMapDouble::iterator distances_iterator = distances_.find(key_);
  if(distances_iterator != distances_.end())
  {
    return distances_iterator->second;
  }
  return max_occ_dist_;
}

std::size_t OctoMap::makeHash(const std::vector<int>& key)
{
  std::size_t hash(0);
  boost::hash_combine(hash, key[0]);
  boost::hash_combine(hash, key[1]);
  boost::hash_combine(hash, key[2]);
  return hash;
}

bool OctoMap::keysEqual(const std::vector<int>& lhs, const std::vector<int>& rhs)
{
  return lhs[0] == rhs[0] && lhs[1] == rhs[1] && lhs[2] == rhs[2];
}

}  // namespace amcl

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
  octree_ = std::make_shared<octomap::OcTree>(resolution_);
}

// initialize octomap from octree
void OctoMap::initFromOctree(std::shared_ptr<octomap::OcTree> octree, double max_occ_dist)
{
  octree_ = octree;
  max_occ_dist_ = max_occ_dist;
  max_occ_dist_ratio_ = max_occ_dist_ / UINT8_MAX;
  double min_x, min_y, min_z, max_x, max_y, max_z;
  octree_->getMetricMin(min_x, min_y, min_z);
  octree_->getMetricMax(max_x, max_y, max_z);
  // crop values here if required
  convertWorldToMap({ min_x, min_y, min_z }, &cropped_min_cells_);
  convertWorldToMap({ max_x, max_y, max_z }, &cropped_max_cells_);
  map_cells_width_ = cropped_max_cells_[0] - cropped_min_cells_[0] + 1;
  num_poses_ = map_cells_width_ * (cropped_max_cells_[1] - cropped_min_cells_[1] + 1);
  num_z_column_indices_ = cropped_max_cells_[2] - cropped_min_cells_[2] + 1;
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
bool OctoMap::isPoseValid(const int i, const int j)
{
  return (i <= cropped_max_cells_[0] and i >= cropped_min_cells_[0]
          and j <= cropped_max_cells_[1] and j >= cropped_min_cells_[1]);
}

bool OctoMap::isVoxelValid(const int i, const int j, const int k)
{
  return isPoseValid(i, j) and k <= cropped_max_cells_[2] and k >= cropped_min_cells_[2];
}

double OctoMap::getMaxOccDist()
{
  return max_occ_dist_;
}

void OctoMap::setMapBounds(const std::vector<double>& map_min, const std::vector<double>& map_max)
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
  map_cells_width_ = cropped_max_cells_[0] - cropped_min_cells_[0] + 1;
  num_poses_ = map_cells_width_ * (cropped_max_cells_[1] - cropped_min_cells_[1] + 1);
  num_z_column_indices_ = cropped_max_cells_[2] - cropped_min_cells_[2] + 1;
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
        double distance = std::sqrt(i * i + j * j + k * k) * resolution;
        cached_distances_[i][j][k] = distance;
      }
    }
  }
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
  pose_indices_.clear();
  pose_indices_.resize(num_poses_, 0);
  pose_indices_.shrink_to_fit();
  distance_ratios_.clear();
  distance_ratios_.resize(num_z_column_indices_, UINT8_MAX);
  distance_ratios_.shrink_to_fit();
  if ((cdm_.resolution_ != resolution_) || (std::fabs(cdm_.max_dist_ - max_occ_dist_) > EPSILON))
  {
    cdm_ = CachedDistanceOctoMap(resolution_, max_occ_dist_);
  }
  ROS_INFO("Iterating obstacle cells");
  iterateObstacleCells(q);
  ROS_INFO("Iterating empty cells");
  iterateEmptyCells(q);
  ROS_INFO("Done updating OctoMap CSpace");
  cspace_created_ = true;
}

void OctoMap::iterateObstacleCells(CellDataQueue& q)
{
  // Enqueue all the obstacle cells
  OctoMapCellData cell = OctoMapCellData();
  std::vector<double> world_coords(3);
  std::vector<int> map_coords(3);

  std::priority_queue<std::vector<int>> ordering_queue;
  std::vector<int> source(3);

  octree_->expand();
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
      if(!isVoxelValid(i, j, k))
        continue;
      setOccDist(i, j, k, 0.0);
      source[0] = i;
      source[1] = j;
      source[2] = k;
      ordering_queue.push(source);
    }
  }

  while(!ordering_queue.empty())
  {
    source = ordering_queue.top();
    ordering_queue.pop();
    cell.src_i = cell.i = source[0];
    cell.src_j = cell.j = source[1];
    cell.src_k = cell.k = source[2];
    q.push(cell);
  }
}

void OctoMap::iterateEmptyCells(CellDataQueue& q)
{
  std::size_t count = 0;
  while (!q.empty())
  {
    OctoMapCellData current_cell = q.front();
    if (current_cell.i > cropped_min_cells_[0])
    {
      enqueue(0, current_cell, q);
    }
    if (current_cell.j > cropped_min_cells_[1])
    {
      enqueue(1, current_cell, q);
    }
    if (current_cell.k > cropped_min_cells_[2])
    {
      enqueue(2, current_cell, q);
    }
    if (current_cell.i < cropped_max_cells_[0])
    {
      enqueue(3, current_cell, q);
    }
    if (current_cell.j < cropped_max_cells_[1])
    {
      enqueue(4, current_cell, q);
    }
    if (current_cell.k < cropped_max_cells_[2])
    {
      enqueue(5, current_cell, q);
    }
    q.pop();
    count = std::max(count, q.size());
  }
  ROS_INFO("Max queue size: %lu", count);
}

// Adds the voxel to the queue if the voxel is close enough to an object
void OctoMap::enqueue(const int shift_index, const OctoMapCellData& current_cell, CellDataQueue& q)
{
  int i = current_cell.i + SHIFTS[shift_index][0];
  int j = current_cell.j + SHIFTS[shift_index][1];
  int k = current_cell.k + SHIFTS[shift_index][2];
  int di = std::abs(i - current_cell.src_i);
  int dj = std::abs(j - current_cell.src_j);
  int dk = std::abs(k - current_cell.src_k);
  double new_distance = cdm_.cached_distances_[di][dj][dk];
  double old_distance = getOccDist(i, j, k);
  if (old_distance - new_distance > max_occ_dist_ratio_)
  {
    setOccDist(i, j, k, new_distance);
    OctoMapCellData cell = OctoMapCellData();
    cell.i = i;
    cell.j = j;
    cell.k = k;
    cell.src_i = current_cell.src_i;
    cell.src_j = current_cell.src_j;
    cell.src_k = current_cell.src_k;
    q.push(cell);
  }
}

// Sets the distance from the voxel to the nearest object in the static map
void OctoMap::setOccDist(int i, int j, int k, double d)
{
  int i_shifted = i - cropped_min_cells_[0];
  int j_shifted = j - cropped_min_cells_[1];
  int k_shifted = k - cropped_min_cells_[2];
  uint32_t pose_index = makePoseIndex(i_shifted, j_shifted);
  uint32_t distances_start_index = pose_indices_[pose_index];
  if(distances_start_index == 0)
  {
    distances_start_index = distance_ratios_.size();
    pose_indices_[pose_index] = distances_start_index;
    distance_ratios_.resize(distances_start_index + num_z_column_indices_, UINT8_MAX);
  }
  uint8_t distance_ratio = std::min(UINT8_MAX, static_cast<int>(std::floor(d / max_occ_dist_ * (UINT8_MAX + 1))));
  distance_ratios_[distances_start_index + k_shifted] = distance_ratio;
}

// returns the distance from the 3d voxel to the nearest object in the static map
double OctoMap::getOccDist(int i, int j, int k)
{
  // Checking if cspace is created first will prevent checking validity while creating the cspace.
  // The cspace is assumed to not send invalid coordinates and checking every time is inefficient.
  if(cspace_created_ and !isVoxelValid(i, j, k))
    return max_occ_dist_;
  int i_shifted = i - cropped_min_cells_[0];
  int j_shifted = j - cropped_min_cells_[1];
  int k_shifted = k - cropped_min_cells_[2];
  uint32_t pose_index = makePoseIndex(i_shifted, j_shifted);
  uint32_t distances_start_index = pose_indices_[pose_index];
  uint8_t distance_ratio = distance_ratios_[distances_start_index + k_shifted];
  double distance = distance_ratio * max_occ_dist_ratio_;
  return distance;
}

uint32_t OctoMap::makePoseIndex(int i, int j)
{
  return j * map_cells_width_ + i;
}

}  // namespace amcl

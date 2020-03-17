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

#include <ros/console.h>

#include <algorithm>
#include <cmath>

using namespace amcl;

OctoMap::OctoMap(double resolution, bool wait_for_occupancy_map) :
    Map(resolution),
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
  full_cells_[0] = (int)ceil(x_meters / resolution_);
  full_cells_[1] = (int)ceil(y_meters / resolution_);
  full_cells_[2] = (int)ceil(z_meters / resolution_);
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
  (*map_coords)[0] = floor((x - origin_.x) / resolution_ + 0.5);
  (*map_coords)[1] = floor((y - origin_.y) / resolution_ + 0.5);
  if (world_coords.size() > 2)
  {
    double z = world_coords[2];
    (*map_coords)[2] = floor((z - origin_.z) / resolution_ + 0.5);
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

// computes the index of the cell for a flattened 3D list
unsigned int OctoMap::computeCellIndex(int i, int j, int k)
{
  return i + j * unsigned(full_cells_[0]) + k * unsigned(full_cells_[1]) * unsigned(full_cells_[2]);
}

double OctoMap::getMaxOccDist()
{
  return max_occ_dist_;
}

void OctoMap::setMapBounds(std::shared_ptr<std::vector<double>> map_min,
                           std::shared_ptr<std::vector<double>> map_max)
{
  std::vector<int> cells_min(map_min->size()), cells_max(map_max->size());
  std::vector<double> map_min_local(*map_min), map_max_local(*map_max);
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
  cell_radius_ = (int)floor(max_dist / resolution);
  distances_.resize(cell_radius_ + 2);
  for (int i = 0; i <= cell_radius_ + 1; i++)
  {
    distances_[i].resize(cell_radius_ + 2);
    for (int j = 0; j <= cell_radius_ + 1; j++)
    {
      distances_[i][j].resize(cell_radius_ + 2);
      for (int k = 0; k <= cell_radius_ + 1; k++)
      {
        distances_[i][j][k] = sqrt(i * i + j * j + k * k);
      }
    }
  }
}

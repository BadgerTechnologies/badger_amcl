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

#include <vector>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeDistance.h>
#include <math.h>
#include "map.h"
#include "octomap.h"

#include "ros/ros.h"

using namespace amcl;

OctoMap::OctoMap()
{
  origin_ = std::vector<double>(3);
  full_cells_ = std::vector<int>(3);
  cropped_min_cells_ = std::vector<int>(3);
  cropped_max_cells_ = std::vector<int>(3);

  max_occ_dist_ = 0.0;

  octree_ = new octomap::OcTree(scale_);
  distances_ = nullptr;
  cdm_ = nullptr;
}

OctoMap::~OctoMap()
{
  delete octree_;
}

// initialize octomap from octree
void
OctoMap::initFromOctree(octomap::OcTree* octree, double lidar_height)
{
  octree_ = octree;
  lidar_height_ = lidar_height;
  // set size
  double x_meters, y_meters, z_meters;
  octree_->getMetricSize(x_meters, y_meters, z_meters);
  full_cells_[0] = (int)ceil(x_meters/scale_);
  full_cells_[1] = (int)ceil(y_meters/scale_);
  full_cells_[2] = (int)ceil(z_meters/scale_);
  double min_x, min_y, min_z, max_x, max_y, max_z;
  octree_->getMetricMin(min_x, min_y, min_z);
  octree_->getMetricMax(max_x, max_y, max_z);
  setOrigin({min_x, min_y, min_z});
  // crop values here if required
  // TODO: crop to 2d map size
  cropped_min_cells_ = convertWorldToMap({min_x, min_y, min_z});
  cropped_max_cells_ = convertWorldToMap({max_x, max_y, max_z});
  // create occ states
  updateCSpace();
}

// getter and setter for global origin of octomap
std::vector<double>
OctoMap::getOrigin()
{
  return origin_;
}

void
OctoMap::setOrigin(std::vector<double> origin)
{
  origin_ = origin;
}

// returns vector of map size in voxels
// each voxel represents map_size_in_meters / resolution
std::vector<int>
OctoMap::getSize()
{
  return full_cells_;
}

void
OctoMap::getMinMaxCells(std::vector<int> min_cells, std::vector<int> max_cells)
{
  min_cells = cropped_min_cells_;
  max_cells = cropped_max_cells_;
}

// converts map voxel coordinates to global coordinates in meters
std::vector<double>
OctoMap::convertMapToWorld(std::vector<int> map_coords)
{
  std::vector<double> return_vals;
  int i = map_coords[0];
  int j = map_coords[1];
  return_vals.push_back(origin_[0] + i * scale_);
  return_vals.push_back(origin_[1] + j * scale_);
  if(map_coords.size() > 2)
  {
    int k = map_coords[2];
    return_vals.push_back(origin_[2] + k * scale_);
  }
  return return_vals;
}

// converts global coordinates in meters to map voxel coordinates
std::vector<int>
OctoMap::convertWorldToMap(std::vector<double> world_coords)
{
  std::vector<int> return_vals;
  double x = world_coords[0];
  double y = world_coords[1];
  return_vals.push_back(floor((x - origin_[0]) / scale_ + 0.5));
  return_vals.push_back(floor((y - origin_[1]) / scale_ + 0.5));
  if(world_coords.size() > 2)
  {
    double z = world_coords[2];
    return_vals.push_back(floor((z - origin_[2]) / scale_ + 0.5));
  }
  return return_vals;
}

// returns true if all coordinates are within the represented map
bool
OctoMap::isValid(std::vector<int> coords)
{
  int i = coords[0];
  int j = coords[1];
  if((i < cropped_min_cells_[0]) || (i >= cropped_max_cells_[0])
     || (j < cropped_min_cells_[1]) || (j >= cropped_max_cells_[1]))
    return false;
  if(coords.size() == 2)
    return true;
  else
  {
    int k = coords[2];
    return (k >= cropped_min_cells_[2]) && (k < cropped_max_cells_[2]);
  }
}

// computes the index of the cell for a flattened 3D list
unsigned int
OctoMap::computeCellIndex(int i, int j, int k)
{
  return i + j * unsigned(full_cells_[0]) + k * unsigned(full_cells_[1]) * unsigned(full_cells_[2]);
}

double OctoMap::getMaxOccDist()
{
  return max_occ_dist_;
}

/*
 *
 *       AMCL Octomap Class
 *        by Tyler Buchman
 *        2019
 *
 */

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
  origin_x = 0;
  origin_y = 0;
  origin_z = 0;

  cells_x = 0;
  cells_y = 0;
  cells_z = 0;

  min_cells_x = 0;
  min_cells_y = 0;
  min_cells_z = 0;
  max_cells_x = 0;
  max_cells_y = 0;
  max_cells_z = 0;

  max_occ_dist = 0.0;

  octree = new octomap::OcTree(scale);
  distances = nullptr;
  cdm = nullptr;
}

OctoMap::~OctoMap()
{
  delete octree;
}

void
OctoMap::initFromOctree(octomap::OcTree* octree, double lidar_height)
{
  this->octree = octree;
  this->lidar_height = lidar_height;
  // set size
  double x_meters, y_meters, z_meters;
  octree->getMetricSize(x_meters, y_meters, z_meters);
  cells_x = (int)ceil(x_meters/scale);
  cells_y = (int)ceil(y_meters/scale);
  cells_z = (int)ceil(z_meters/scale);
  double min_x, min_y, min_z, max_x, max_y, max_z;
  octree->getMetricMin(min_x, min_y, min_z);
  octree->getMetricMax(max_x, max_y, max_z);
  setOrigin({min_x, min_y, min_z});

  ROS_INFO("metric min: %f, %f, %f", min_x, min_y, min_z);
  ROS_INFO("metric max: %f, %f, %f", max_x, max_y, max_z);
  double my_min_x, my_min_y, my_min_z, my_max_x, my_max_y, my_max_z;
  my_min_x = std::max(0.0, min_x);
  my_min_y = std::max(0.0, min_y);
  my_min_z = std::max(0.0, min_z);
  my_max_x = std::min(34.0, max_x);
  my_max_y = std::min(31.0, max_y);
  my_max_z = std::min(3.6, max_z);
  std::vector<int> min_pix = convertWorldToMap({my_min_x, my_min_y, my_min_z});
  std::vector<int> max_pix = convertWorldToMap({my_max_x, my_max_y, my_max_z});
  min_cells_x = min_pix[0];
  min_cells_y = min_pix[1];
  min_cells_z = min_pix[2];
  max_cells_x = max_pix[0];
  max_cells_y = max_pix[1];
  max_cells_z = max_pix[2];
  ROS_INFO("minx: %d, miny: %d, minz: %d, maxx: %d, maxy: %d, maxz: %d", min_cells_x, min_cells_y, min_cells_z, max_cells_x, max_cells_y, max_cells_z);
  map_min = octomap::point3d(my_min_x, my_min_y, my_min_z);
  map_max = octomap::point3d(my_max_x, my_max_y, my_max_z);
  // create occ states
  updateCSpace();
}

std::vector<double>
OctoMap::getOrigin()
{
  return {origin_x, origin_y, origin_z};
}

void
OctoMap::setOrigin(std::vector<double> _origin)
{
  origin_x = _origin[0];
  origin_y = _origin[1];
  origin_z = _origin[2];
}

std::vector<int>
OctoMap::getSize()
{
  return {cells_x, cells_y, cells_z};
}

std::vector<double>
OctoMap::convertMapToWorld(std::vector<int> map_coords)
{
  std::vector<double> return_vals;
  int i = map_coords[0];
  int j = map_coords[1];
  return_vals.push_back(origin_x + i * scale);
  return_vals.push_back(origin_y + j * scale);
  if(map_coords.size() > 2)
  {
    int k = map_coords[2];
    return_vals.push_back(origin_z + (k - cells_z / 2) * scale);
  }
  return return_vals;
}

std::vector<int>
OctoMap::convertWorldToMap(std::vector<double> world_coords)
{
  std::vector<int> return_vals;
  double x = world_coords[0];
  double y = world_coords[1];
  return_vals.push_back(floor((x - origin_x) / scale + 0.5));
  return_vals.push_back(floor((y - origin_y) / scale + 0.5));
  if(world_coords.size() > 2)
  {
    double z = world_coords[2];
    return_vals.push_back(floor((z - origin_z) / scale + 0.5));
  }
  return return_vals;
}

bool
OctoMap::isValid(std::vector<int> coords)
{
  int i = coords[0];
  int j = coords[1];
  if((i < min_cells_x) || (i >= max_cells_x)
     || (j < min_cells_y) || (j >= max_cells_y))
    return false;
  if(coords.size() == 2)
    return true;
  else
  {
    int k = coords[2];
    return (k >= min_cells_z) && (k < max_cells_z);
  }
}

unsigned int
OctoMap::computeCellIndex(int i, int j, int k)
{
  return i + j * unsigned(cells_x) + k * unsigned(cells_x) * unsigned(cells_y);
}

double OctoMap::getMaxOccDist()
{
  return max_occ_dist;
}

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
#include "map.h"
#include "octomap.h"

using namespace amcl;

OctoMap::OctoMap()
{
  origin_x = 0;
  origin_y = 0;
  origin_z = 0;

  size_x = 0;
  size_y = 0;
  size_z = 0;

  octree = new octomap::OcTree(scale);
  
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
  return {size_x, size_y, size_z};
}

void
OctoMap::setSize(std::vector<int> _size)
{
  size_x = _size[0];
  size_y = _size[1];
  size_z = _size[2];
}

std::vector<double>
OctoMap::convertMapToWorld(std::vector<int> map_coords)
{
  std::vector<double> return_vals;
  int i = map_coords[0];
  int j = map_coords[1];
  int k = map_coords[2];
  return_vals.push_back(origin_x + (i - size_x / 2) * scale);
  return_vals.push_back(origin_y + (j - size_y / 2) * scale);
  return_vals.push_back(origin_z + (k - size_z / 2) * scale);
  return return_vals;
}

std::vector<int>
OctoMap::convertWorldToMap(std::vector<double> world_coords)
{
  std::vector<int> return_vals;
  double x = world_coords[0];
  double y = world_coords[1];
  double z = world_coords[2];
  return_vals.push_back(floor((x - origin_x) / scale + 0.5) + size_x / 2);
  return_vals.push_back(floor((y - origin_y) / scale + 0.5) + size_y / 2);
  return_vals.push_back(floor((z - origin_z) / scale + 0.5) + size_z / 2);
  return return_vals;
}

bool
OctoMap::isValid(std::vector<int> coords)
{
  int i = coords[0];
  int j = coords[1];
  int k = coords[2];
  return (i >= 0) && (i < size_x) && (j >= 0) && (j < size_y) && (k >= 0) && (k < size_z);
}

/*
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

#ifndef AMCL_MAP_MAP_H
#define AMCL_MAP_MAP_H

#include <atomic>
#include <vector>

#include <pcl/point_types.h>

namespace badger_amcl
{

class Map
{
public:
  Map(double resolution);
  virtual ~Map() = default;

  // Convert from map index to world coords
  virtual void convertMapToWorld(const std::vector<int>& map_coords, std::vector<double>* world_coords) = 0;
  // Convert from world coords to map coords
  virtual void convertWorldToMap(const std::vector<double>& world_coords, std::vector<int>* map_coords) = 0;

  virtual bool isDistancesLUTCreated();
  virtual pcl::PointXYZ getOrigin();

protected:
  // Map origin; the map is a viewport onto a conceptual larger map.
  pcl::PointXYZ origin_;
  double resolution_;
  // Max distance at which we care about obstacles, for constructing
  // likelihood field
  double max_distance_to_object_;
  std::atomic<bool> distances_lut_created_;
};
}  // namespace amcl

#endif  // AMCL_MAP_MAP_H

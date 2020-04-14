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

#include "map/map.h"

namespace badger_amcl
{

// Create a new map
Map::Map(double resolution) : resolution_(resolution)
{
  origin_ = pcl::PointXYZ();
  cspace_created_ = false;
}

pcl::PointXYZ Map::getOrigin()
{
  return origin_;
}

void Map::setOrigin(const pcl::PointXYZ& origin)
{
  origin_ = origin;
}

bool Map::isCSpaceCreated()
{
  return cspace_created_;
}

}  // namespace amcl

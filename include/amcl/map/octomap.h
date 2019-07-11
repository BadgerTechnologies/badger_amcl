/*
 *
 *       AMCL Octomap Class
 *        by Tyler Buchman
 *        2019
 *
 */

#ifndef OCTOMAP_H
#define OCTOMAP_H

#include <vector>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include "map.h"

namespace amcl
{

class OctoMap : public Map
{
  public:
    OctoMap();
    ~OctoMap();
    // Convert from map index to world coords
    std::vector<double> convertMapToWorld(std::vector<int> map_coords);
    // Convert from world coords to map coords
    std::vector<int> convertWorldToMap(std::vector<double> world_coords);
    // Test to see if the given map coords lie within the absolute map bounds.
    bool isValid(std::vector<int> coords);
    std::vector<double> getOrigin();
    void setOrigin(std::vector<double> _origin);
    std::vector<int> getSize();
    void setSize(std::vector<int> _size);
  private:
    octomap::OcTree* octree;
    // Map origin; the map is a viewport onto a conceptual larger map.
    double origin_x, origin_y, origin_z;
    // Map dimensions (number of cells)
    int size_x, size_y, size_z;
};

}

#endif

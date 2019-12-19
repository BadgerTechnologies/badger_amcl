/*
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
///////////////////////////////////////////////////////////////////////////
//
// Desc: OctoMap for 3D AMCL
// Author: Tyler Buchman (tyler_buchman@jabil.com)
//
///////////////////////////////////////////////////////////////////////////



#ifndef AMCL_OCTOMAP_H
#define AMCL_OCTOMAP_H

#include "map/map.h"

#include <octomap/OcTree.h>
#include <octomap/OcTreeDistance.h>

#include <limits>
#include <memory>
#include <queue>
#include <vector>

namespace amcl
{

class OctoMap : public Map
{
  public:
    OctoMap(bool wait_for_occupancy_map);
    ~OctoMap(){};
    // Convert from map index to world coords
    void convertMapToWorld(const std::vector<int> &map_coords, std::vector<double> *world_coords);
    // Convert from world coords to map coords
    void convertWorldToMap(const std::vector<double> &world_coords, std::vector<int> *map_coords);
    // Test to see if the given map coords lie within the absolute map bounds.
    bool isValid(std::vector<int> coords);
    std::vector<int> getSize();
    void getMinMaxCells(std::vector<int> *min_cells, std::vector<int> *max_cells);
    std::vector<double> getOrigin();
    void setOrigin(std::vector<double> origin);
    void setMapBounds(std::vector<double> map_min, std::vector<double> map_max);
    // Update the cspace distance values
    void updateCSpace();
    void updateMaxOccDist(double max_occ_dist);
    void initFromOctree(std::shared_ptr<octomap::OcTree> octree, double lidar_height);
    double getOccDist(int i, int j, int k);
    double getOccDist(int i, int j);
    double getMaxOccDist();
  private:
    const double EPSILON_DOUBLE = std::numeric_limits<double>::epsilon();
    struct CellData {
      OctoMap* octoMap;
      CellData(OctoMap& _octoMap) : octoMap(&_octoMap){}
      int i_, j_, k_;
      int src_i_, src_j_, src_k_;
    };

    class CachedDistanceMap
    {
      public:
        double*** distances_;
        double scale_;
        double max_dist_;
        int cell_radius_;

        CachedDistanceMap(double scale, double max_dist) :
          distances_(nullptr), scale_(scale), max_dist_(max_dist)
        {
          cell_radius_ = max_dist / scale;
          distances_ = new double **[cell_radius_+2];
          for(int i=0; i <= cell_radius_+1; i++)
          {
            distances_[i] = new double *[cell_radius_+2];
            for(int j = 0; j <= cell_radius_+1; j++)
            {
              distances_[i][j] = new double[cell_radius_+2];
              for(int k=0; k <= cell_radius_+1; k++)
              {
                distances_[i][j][k] = sqrt(i*i + j*j + k*k);
              }
            }
          }
        }

        ~CachedDistanceMap()
        {
          if(distances_)
          {
            for(int i=0; i<=cell_radius_+1; i++)
            {
              for(int j=0; j<=cell_radius_+1; j++)
              {
                delete[] distances_[i][j];
              }
              delete[] distances_[i];
            }
            delete[] distances_;
          }
        }
    };

    void setOccDist(int i, int j, int k, double d);
    bool enqueue(int i, int j, int k, int src_i, int src_j, int src_k,
	             std::priority_queue<CellData>& Q);
    unsigned int computeCellIndex(int i, int j, int k);

    friend bool operator<(const OctoMap::CellData& a, const OctoMap::CellData& b);

    std::shared_ptr<octomap::OcTree> octree_;
    std::unique_ptr<octomap::OcTreeDistance> distances_;
    // Map origin; the map is a viewport onto a conceptual larger map.
    std::vector<double> origin_;
    // Map dimensions (number of cells)
    std::vector<double> map_min_bounds_, map_max_bounds_;
    std::vector<int> cropped_min_cells_, cropped_max_cells_, full_cells_;
    double lidar_height_;
    bool wait_for_occupancy_map_;
    std::unique_ptr<CachedDistanceMap> cdm_;
};

inline bool operator<(const OctoMap::CellData& a, const OctoMap::CellData& b)
{
  return a.octoMap->getOccDist(a.i_, a.j_, a.k_) > b.octoMap->getOccDist(b.i_, b.j_, b.k_);
}

}

#endif

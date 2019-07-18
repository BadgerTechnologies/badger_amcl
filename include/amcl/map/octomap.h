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
#include <octomap/OcTreeDistance.h>
#include <octomap/OcTreeKey.h>
#include <octomap/OcTreeDataNode.h>
#include <queue>
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
    std::vector<int> getSize();
    std::vector<double> getOrigin();
    void setOrigin(std::vector<double> _origin);
    // Update the cspace distance values
    void updateCSpace();
    void updateCSpace(double max_occ_dist_);
    void initFromOctree(octomap::OcTree* octree, double lidar_height);
    double getOccDist(int i, int j, int k, bool pub);
    double getOccDist(int i, int j, int k);
    double getOccDist(int i, int j);
    double getMaxOccDist();
  private:
    const double EPSILON_DOUBLE = std::numeric_limits<double>::epsilon();
    struct CellData {
      OctoMap* octoMap;
      CellData(OctoMap* _octoMap) : octoMap(_octoMap){}
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

    octomap::OcTree* octree;
    octomap::OcTreeDistance* distances;
    // Map origin; the map is a viewport onto a conceptual larger map.
    double origin_x, origin_y, origin_z;
    // Map dimensions (number of cells)
    int cells_x, cells_y, cells_z, min_cells_x, min_cells_y, min_cells_z, max_cells_x, max_cells_y, max_cells_z;
    octomap::point3d map_min, map_max;
    double lidar_height;

    CachedDistanceMap* cdm;

    void setOccDist(int i, int j, int k, double d);
    bool enqueue(int i, int j, int k, int src_i, int src_j, int src_k,
	             std::priority_queue<CellData>& Q);
    unsigned int computeCellIndex(int i, int j, int k);

    friend bool operator<(const OctoMap::CellData& a, const OctoMap::CellData& b);

};

inline bool operator<(const OctoMap::CellData& a, const OctoMap::CellData& b)
{
  return a.octoMap->getOccDist(a.i_, a.j_, a.k_) > b.octoMap->getOccDist(b.i_, b.j_, b.k_);
}

}

#endif

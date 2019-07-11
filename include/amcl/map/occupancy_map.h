/*
 *  Player - One Hell of a Robot Server
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
/**************************************************************************
 * Desc: Global map (grid-based)
 * Author: Andrew Howard
 * Date: 6 Feb 2003
 * CVS: $Id: map.h 1713 2003-08-23 04:03:43Z inspectorg $
 **************************************************************************/

#ifndef OCCUPANCY_MAP_H
#define OCCUPANCY_MAP_H

#include <cstdint>
#include <vector>
#include <queue>
#include <math.h>
#include "map.h"

namespace amcl
{

// Description for a single map cell.
struct map_cell_t
{
  // Occupancy state (-1 = free, 0 = unknown, +1 = occ)
  int8_t occ_state;
};


class OccupancyMap : public Map
{
  public:
    OccupancyMap();
    ~OccupancyMap();
    // Convert from map index to world coords
    std::vector<double> convertMapToWorld(std::vector<int> map_coords);
    // Convert from world coords to map coords
    std::vector<int> convertWorldToMap(std::vector<double> world_coords);
    // Test to see if the given map coords lie within the absolute map bounds.
    bool isValid(std::vector<int> coords);
    std::vector<double> getOrigin();
    void setOrigin(std::vector<double> _origin);
    std::vector<int> getSize();
    void setSize(std::vector<int> _size_x);
    void updateCSpace(double max_occ_dist_);
    // Extract a single range reading from the map
    double calcRange(double ox, double oy, double oa, double max_range);
    // Find the distance to nearest occupied cell
    float occDist(int i, int j);
    // Compute the cell index for the given map coords.
    unsigned int computeCellIndex(int i, int j);
    double getMaxOccDist();
    map_cell_t* getCells();
    void initCells(int num);
    void setCellOccState(int index, int8_t state);

    struct CellData {
      OccupancyMap* occMap;
      CellData(OccupancyMap* _occMap) : occMap(_occMap){}
      int i_, j_;
      int src_i_, src_j_;

    };

  private:
    class CachedDistanceMap
    {
      public:
        double** distances_;
        double scale_;
        double max_dist_;
        int cell_radius_;

        CachedDistanceMap(double scale, double max_dist) :
          distances_(nullptr), scale_(scale), max_dist_(max_dist)
        {
          cell_radius_ = max_dist / scale;
          distances_ = new double *[cell_radius_+2];
          for(int i=0; i<=cell_radius_+1; i++)
          {
            distances_[i] = new double[cell_radius_+2];
            for(int j=0; j<=cell_radius_+1; j++)
            {
              distances_[i][j] = sqrt(i*i + j*j);
            }
          }
        }

        ~CachedDistanceMap()
        {
          if(distances_)
          {
            for(int i=0; i<=cell_radius_+1; i++)
              delete[] distances_[i];
            delete[] distances_;
          }
        }
    };

    // Map origin; the map is a viewport onto a conceptual larger map.
    double origin_x, origin_y;

    // Map dimensions (number of cells)
    int size_x, size_y;

    // The map occupancy data, stored as a grid
    map_cell_t *cells;

    // The map distance data, stored as a grid
    float *distances;

    // Max distance at which we care about obstacles, for constructing
    // likelihood field
    double max_occ_dist;

    CachedDistanceMap* cdm;

    void setMapOccDist(int i, int j, float d);
    CachedDistanceMap* getDistanceMap(double scale,
                                      double max_dist);
    bool enqueue(int i, int j, int src_i, int src_j,
	             std::priority_queue<CellData>& Q,
	             CachedDistanceMap* cdm);

    friend bool operator<(const OccupancyMap::CellData& a,
                          const OccupancyMap::CellData& b);
};

inline bool operator<(const OccupancyMap::CellData& a,
               const OccupancyMap::CellData& b)
{
  return a.occMap->occDist(a.i_, a.j_) > b.occMap->occDist(b.i_, b.j_);
}

}

#endif

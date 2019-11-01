/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey et al.
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
///////////////////////////////////////////////////////////////////////////
//
// Desc: Point Cloud Scanner sensor model for 3D AMCL
// Author: Tyler Buchman (tyler_buchman@jabil.com)
//
///////////////////////////////////////////////////////////////////////////


#ifndef AMCL_POINT_CLOUD_SCANNER_H
#define AMCL_POINT_CLOUD_SCANNER_H

#include "sensor.h"
#include "../map/map.h"
#include "../map/octomap.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "tf/tf.h"

namespace amcl
{

typedef enum
{
  POINT_CLOUD_MODEL,
} PointCloudModelType;


class PointCloudData : public SensorData
{
  public:
    PointCloudData () {};
    virtual ~PointCloudData() {};
    std::string frame_id_;
    pcl::PointCloud<pcl::PointXYZ> points_;
};

class PointCloudScanner : public Sensor
{
  public:
    PointCloudScanner(size_t max_beams, OctoMap* map, double lidar_height);
    ~PointCloudScanner();

    void setPointCloudModel(double z_hit, double z_rand, double sigma_hit, double max_occ_dist);

    // Update the filter based on the sensor model.  Returns true if the
    // filter has been updated.
    bool updateSensor(ParticleFilter *pf, SensorData *data);

    // Update a sample set based on the sensor model.
    // Returns total weights of particles, or 0.0 on failure.
    static double applyModelToSampleSet(SensorData *data, PFSampleSet *set);

    void setMapFactors(double off_map_factor,
                       double non_free_space_factor,
                       double non_free_space_radius);

    // Set the scanner's pose after construction
    void setPointCloudScannerPose(PFVector& scanner_pose)
    {this->point_cloud_scanner_pose_ = scanner_pose;}

    void setPointCloudScannerToFootprintTF(tf::Transform lidarToFootprintTF);

    int getMaxBeams();

  private:
    // Determine the probability for the given pose
    static double calcPointCloudModel(PointCloudData *data, PFSampleSet *set);

    OctoMap *map_;
    PFVector point_cloud_scanner_pose_;
    PointCloudModelType model_type_;
    double off_map_factor_;
    double non_free_space_factor_;
    double non_free_space_radius_;
    // Mixture params for the components of the model; must sum to 1
    double z_hit_;
    double z_short_;
    double z_max_;
    double z_rand_;
    // Stddev of Gaussian model for lidar hits.
    double sigma_hit_;
    // Max beams to consider
    int max_beams_;
    double point_cloud_scanner_height_;

    tf::Transform point_cloud_scanner_to_footprint_tf_;
};

}

#endif

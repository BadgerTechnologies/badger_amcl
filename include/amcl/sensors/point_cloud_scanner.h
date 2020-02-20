/*
 *  Copyright (C) 2020 Badger Technologies, LLC
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
/* ********************************************************************
 * Desc: Point Cloud Scanner sensor model for 3D AMCL
 * Author: Tyler Buchman (tyler_buchman@jabil.com)
 **********************************************************************/

#ifndef AMCL_SENSORS_POINT_CLOUD_SCANNER_H
#define AMCL_SENSORS_POINT_CLOUD_SCANNER_H

#include "sensors/sensor.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <tf/transform_broadcaster.h>

#include <memory>

#include "map/octomap.h"
#include "pf/particle_filter.h"
#include "pf/pf_vector.h"

namespace amcl
{
typedef enum {
  POINT_CLOUD_MODEL,
  POINT_CLOUD_MODEL_GOMPERTZ,
} PointCloudModelType;

class PointCloudData : public SensorData
{
public:
  PointCloudData(){};
  virtual ~PointCloudData(){};
  std::string frame_id_;
  pcl::PointCloud<pcl::PointXYZ> points_;
};

class PointCloudScanner : public Sensor
{
public:
  PointCloudScanner();
  ~PointCloudScanner(){};

  void init(size_t max_beams, std::shared_ptr<OctoMap> map, double lidar_height);

  void setPointCloudModel(double z_hit, double z_rand, double sigma_hit, double max_occ_dist);
  void setPointCloudModelGompertz(double z_hit, double z_rand, double sigma_hit, double max_occ_dist, double gompertz_a,
                                  double gompertz_b, double gompertz_c, double input_shift, double input_scale,
                                  double output_shift);

  // Update the filter based on the sensor model.  Returns true if the
  // filter has been updated.
  bool updateSensor(std::shared_ptr<ParticleFilter> pf, std::shared_ptr<SensorData> data);

  // Update a sample set based on the sensor model.
  // Returns total weights of particles, or 0.0 on failure.
  double applyModelToSampleSet(std::shared_ptr<SensorData> data, std::shared_ptr<PFSampleSet> set);

  void setMapFactors(double off_map_factor, double non_free_space_factor, double non_free_space_radius);

  // Set the scanner's pose after construction
  void setPointCloudScannerPose(PFVector& scanner_pose);
  void setPointCloudScannerToFootprintTF(tf::Transform lidarToFootprintTF);

  int getMaxBeams();
  double applyGompertz(double p);

private:
  // Determine the probability for the given pose
  static double calcPointCloudModel(std::shared_ptr<PointCloudData> data, std::shared_ptr<PFSampleSet> set);
  static double calcPointCloudModelGompertz(std::shared_ptr<PointCloudData> data, std::shared_ptr<PFSampleSet> set);
  static bool getMapCloud(std::shared_ptr<PointCloudScanner> self, std::shared_ptr<PointCloudData> data, PFVector pose,
                          pcl::PointCloud<pcl::PointXYZ>& map_cloud);

  std::shared_ptr<OctoMap> map_;
  PFVector point_cloud_scanner_pose_;
  PointCloudModelType model_type_;

  // Parameters for applying Gompertz function to sample weights
  double gompertz_a_;
  double gompertz_b_;
  double gompertz_c_;
  double input_shift_;
  double input_scale_;
  double output_shift_;

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

  // Vector to store converted map coordinates.
  // Making this a class variable reduces the number of
  // times we need to create an instance of this vector.
  std::vector<int> map_vec_;
};
}  // namespace amcl

#endif  // AMCL_SENSORS_POINT_CLOUD_SCANNER_H

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

#ifndef AMCL_SENSORS_POINT_CLOUD_SCANNER_H
#define AMCL_SENSORS_POINT_CLOUD_SCANNER_H

#include <memory>

#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>

#include "map/octomap.h"
#include "pf/particle_filter.h"
#include "sensors/sensor.h"

namespace badger_amcl
{

enum PointCloudModelType
{
  POINT_CLOUD_MODEL,
  POINT_CLOUD_MODEL_GOMPERTZ,
};

class PointCloudData : public SensorData
{
public:
  virtual ~PointCloudData() = default;
  std::string frame_id_;
  pcl::PointCloud<pcl::PointXYZ> points_;
};

class PointCloudScanner : public Sensor
{
public:
  PointCloudScanner();
  ~PointCloudScanner() = default;

  void init(int max_beams, std::shared_ptr<OctoMap> map);

  void setPointCloudModel(double z_hit, double z_rand, double sigma_hit);
  void setPointCloudModelGompertz(double z_hit, double z_rand, double sigma_hit, double gompertz_a, double gompertz_b,
                                  double gompertz_c, double input_shift, double input_scale, double output_shift);

  // Update the filter based on the sensor model.  Returns true if the
  // filter has been updated.
  bool updateSensor(std::shared_ptr<ParticleFilter> pf, std::shared_ptr<SensorData> data);

  // Update a sample set based on the sensor model.
  // Returns total weights of particles, or 0.0 on failure.
  double applyModelToSampleSet(std::shared_ptr<SensorData> data, std::shared_ptr<PFSampleSet> set);

  void setMapFactors(double off_map_factor, double non_free_space_factor, double non_free_space_radius);

  // Set the scanner's pose after construction
  void setPointCloudScannerToFootprintTF(geometry_msgs::Transform tf_msg);

  int getMaxBeams();
  double applyGompertz(double p);

private:
  // Determine the probability for the given pose
  double calcPointCloudModel(std::shared_ptr<PointCloudData> data, std::shared_ptr<PFSampleSet> set);
  double calcPointCloudModelGompertz(std::shared_ptr<PointCloudData> data, std::shared_ptr<PFSampleSet> set);
  double recalcWeight(std::shared_ptr<PFSampleSet> set);
  void getMapCloud(std::shared_ptr<PointCloudData> data, const Eigen::Vector3d& pose,
                   pcl::PointCloud<pcl::PointXYZ>& map_cloud);

  std::shared_ptr<OctoMap> map_;
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

  tf2::Transform point_cloud_scanner_to_footprint_tf_;

  // Vector to store converted map coordinates.
  // Making this a class variable reduces the number of
  // times we need to create an instance of this vector.
  std::vector<int> map_vec_;
  std::vector<double> world_vec_;
};

}  // namespace amcl

#endif  // AMCL_SENSORS_POINT_CLOUD_SCANNER_H

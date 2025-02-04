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

#include "sensors/point_cloud_scanner.h"

#include <cmath>
#include <functional>

#include <geometry_msgs/PoseArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/assert.h>
#include <tf/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

namespace badger_amcl
{

PointCloudScanner::PointCloudScanner() : Sensor()
{
  max_beams_ = 0;

  off_map_factor_ = 1.0;
  non_free_space_factor_ = 1.0;
  non_free_space_radius_ = 0.0;

  voxel_.resize(3);
  point_.resize(3);
}

void PointCloudScanner::init(
    int max_beams, std::shared_ptr<OctoMap> map, std::string global_frame_id,
    double z_hit, double z_rand, double sigma_hit,
    double gompertz_a, double gompertz_b, double gompertz_c,
    double input_shift, double input_scale, double output_shift,
    int num_best_fit_particles)
{
  max_beams_ = max_beams;
  map_ = map;
  global_frame_id_ = global_frame_id;
  z_hit_ = z_hit;
  z_rand_ = z_rand;
  sigma_hit_ = sigma_hit;
  gompertz_a_ = gompertz_a;
  gompertz_b_ = gompertz_b;
  gompertz_c_ = gompertz_c;
  input_shift_ = input_shift;
  input_scale_ = input_scale;
  output_shift_ = output_shift;
  num_best_fit_particles_ = num_best_fit_particles;
  best_fit_cloud_pub_ = nh_.advertise<PointCloud>("best_fit_cloud", 1);
  best_fit_particles_pub_ = nh_.advertise<geometry_msgs::PoseArray>("best_fit_particles", 1);
}

void PointCloudScanner::setMapFactors(double off_map_factor, double non_free_space_factor,
                                      double non_free_space_radius)
{
  off_map_factor_ = off_map_factor;
  non_free_space_factor_ = non_free_space_factor;
  non_free_space_radius_ = non_free_space_radius;
}

void PointCloudScanner::setPointCloudScannerToFootprintTF(geometry_msgs::Transform tf_msg)
{
  tf2::fromMsg(tf_msg, point_cloud_scanner_to_footprint_tf_);
}

// Update the filter based on the sensor model.  Returns true if the
// filter has been updated.
bool PointCloudScanner::updateSensor(std::shared_ptr<ParticleFilter> pf,
                                     std::shared_ptr<SensorData> data)
{
  if (max_beams_ < 2)
    return false;
  // Apply the point cloud scanner sensor model
  std::function<double(std::shared_ptr<SensorData>, std::shared_ptr<PFSampleSet>)> sensor_fn = (
          std::bind(&PointCloudScanner::applyModelToSampleSet, this, std::placeholders::_1, std::placeholders::_2));
  pf->updateSensor(sensor_fn, data);
  return true;
}

// Update a sample set based on the sensor model.
// Returns total weights of particles, or 0.0 on failure.
double PointCloudScanner::applyModelToSampleSet(std::shared_ptr<SensorData> data,
                                                std::shared_ptr<PFSampleSet> set)
{
  if (max_beams_ < 2)
    return 0.0;

  double total_weight = calcPointCloudModelGompertz(std::dynamic_pointer_cast<PointCloudData>(data), set);

  // Apply any configured correction factors from map
  if (total_weight > 0.0)
  {
    total_weight = applyOffMapFactor(set);
  }
  return total_weight;
}

double PointCloudScanner::calcPointCloudModelGompertz(std::shared_ptr<PointCloudData> cloud,
                                                      std::shared_ptr<PFSampleSet> set)
{
  double total_weight = 0.0, p, z, pz, sum_pz;
  PFSample* sample;
  Eigen::Vector3d pose;
  double z_hit_denom = 2 * sigma_hit_ * sigma_hit_;
  PointCloud pose_cloud;
  std::vector<PValueIndex> p_values;
  p_values.resize(set->sample_count);
  for (int sample_index = 0; sample_index < set->sample_count; sample_index++)
  {
    sample = &(set->samples[sample_index]);
    pose = sample->pose;
    getPoseCloud(cloud, pose, pose_cloud);
    sum_pz = 0;
    int count = 0;
    for (PointCloud::iterator it = pose_cloud.begin(); it != pose_cloud.end(); ++it)
    {
      point_[0] = it->x;
      point_[1] = it->y;
      point_[2] = it->z;
      map_->rasterize(point_, &voxel_);
      z = map_->getDistanceToObject(voxel_[0], voxel_[1], voxel_[2]);
      pz = z_hit_ * std::exp(-(z * z) / z_hit_denom);
      pz += z_rand_;
      sum_pz += pz;
      count++;
    }
    p = sum_pz / count;
    p_values[sample_index] = PValueIndex(p, sample_index);
    p = applyGompertz(p);
    sample->weight *= p;
    total_weight += sample->weight;
  }
  std::sort(p_values.begin(), p_values.end(), [] (const PValueIndex &a, const PValueIndex &b)
                                                  {return a.first > b.first;});
  sample = &(set->samples[p_values.front().second]);
  pose = sample->pose;
  getPoseCloud(cloud, pose, pose_cloud);
  PointCloud::Ptr pub_cloud(new PointCloud);
  pub_cloud->header.frame_id = global_frame_id_;
  pub_cloud->height = pose_cloud.height;
  pub_cloud->width = pose_cloud.width;
  pub_cloud->points = pose_cloud.points;
  pcl_conversions::toPCL(ros::Time::now(), pub_cloud->header.stamp);
  best_fit_cloud_pub_.publish(*pub_cloud);
  geometry_msgs::PoseArray best_fit_particles_msg;
  best_fit_particles_msg.header.stamp = ros::Time::now();
  best_fit_particles_msg.header.frame_id = global_frame_id_;
  best_fit_particles_msg.poses.resize(num_best_fit_particles_);
  tf2::Quaternion q;
  for (int i = 0; i < num_best_fit_particles_; i++)
  {
    sample = &(set->samples[p_values[i].second]);
    q.setRPY(0.0, 0.0, sample->pose[2]);
    tf2::toMsg(tf2::Transform(q, tf2::Vector3(sample->pose[0], sample->pose[1], 0)),
               best_fit_particles_msg.poses[i]);
  }
  best_fit_particles_pub_.publish(best_fit_particles_msg);
  return total_weight;
}

double PointCloudScanner::applyOffMapFactor(std::shared_ptr<PFSampleSet> set)
{
  PFSample* sample;
  Eigen::Vector3d pose;
  double total_weight = 0.0;
  int j;
  for (j = 0; j < set->sample_count; j++)
  {
    sample = &(set->samples[j]);
    pose = sample->pose;

    // Convert to map grid coords.
    point_[0] = pose[0];
    point_[1] = pose[1];
    map_->rasterize(point_, &voxel_);

    // Apply off map factor
    if (!map_->isPoseValid(voxel_[0], voxel_[1]))
    {
      sample->weight *= off_map_factor_;
    }
    total_weight += sample->weight;
  }
  return total_weight;
}

void PointCloudScanner::getPoseCloud(std::shared_ptr<PointCloudData> cloud, const Eigen::Vector3d& pose,
                                     pcl::PointCloud<pcl::PointXYZ>& pose_cloud)
{
  tf2::Vector3 footprint_to_map_origin(pose[0], pose[1], 0.0);
  tf2::Quaternion footprint_to_map_q;
  footprint_to_map_q.setRPY(0.0, 0.0, pose[2]);
  tf2::Transform footprint_to_map_tf(footprint_to_map_q, footprint_to_map_origin);
  tf2::Transform t = footprint_to_map_tf * point_cloud_scanner_to_footprint_tf_;
  tf2::Stamped<tf2::Transform> t_s(t, ros::Time::now(), cloud->frame_id_);
  geometry_msgs::TransformStamped tf_msg = tf2::toMsg(t_s);
  pcl::PCLPointCloud2 pcl2;
  pcl::toPCLPointCloud2(cloud->points_, pcl2);
  sensor_msgs::PointCloud2 in_msg, out_msg;
  pcl_conversions::fromPCL(pcl2, in_msg);
  tf2::doTransform(in_msg, out_msg, tf_msg);
  pcl_conversions::toPCL(out_msg, pcl2);
  pcl::fromPCLPointCloud2(pcl2, pose_cloud);
}

double PointCloudScanner::applyGompertz(double p)
{
  // shift and scale p
  p = p * input_scale_ + input_shift_;
  // apply gompertz
  p = gompertz_a_ * std::exp(-1.0 * gompertz_b_ * std::exp(-1.0 * gompertz_c_ * p));
  // shift output
  p += output_shift_;

  return p;
}

int PointCloudScanner::getMaxBeams()
{
  return max_beams_;
}

}  // namspace amcl

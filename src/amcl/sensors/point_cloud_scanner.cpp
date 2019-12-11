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


#include "point_cloud_scanner.h"
#include "ros/ros.h"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>


using namespace amcl;

PointCloudScanner::PointCloudScanner(size_t max_beams, OctoMap* map, double point_cloud_scanner_height) : Sensor()
{
  this->max_beams_ = max_beams;
  this->map_ = map;
  this->point_cloud_scanner_height_ = point_cloud_scanner_height;

  this->off_map_factor_ = 1.0;
  this->non_free_space_factor_ = 1.0;
  this->non_free_space_radius_ = 0.0;

  if(this->map_vec_.size() != 3)
    this->map_vec_ = {0, 0, 0};
}

PointCloudScanner::~PointCloudScanner()
{
  delete map_;
}

void
PointCloudScanner::setPointCloudModel(double z_hit, double z_rand, double sigma_hit, double max_occ_dist)
{
  this->model_type_ = POINT_CLOUD_MODEL;
  this->z_hit_ = z_hit;
  this->z_rand_ = z_rand;
  this->sigma_hit_ = sigma_hit;
  this->map_->updateMaxOccDist(max_occ_dist);
}

void
PointCloudScanner::setPointCloudModelGompertz(double z_hit, double z_rand, double sigma_hit, double max_occ_dist,
                                              double gompertz_a, double gompertz_b, double gompertz_c,
                                              double input_shift, double input_scale, double output_shift)
{
  this->model_type_ = POINT_CLOUD_MODEL_GOMPERTZ;
  this->z_hit_ = z_hit;
  this->z_rand_ = z_rand;
  this->sigma_hit_ = sigma_hit;
  this->gompertz_a_ = gompertz_a;
  this->gompertz_b_ = gompertz_b;
  this->gompertz_c_ = gompertz_c;
  this->input_shift_ = input_shift;
  this->input_scale_ = input_scale;
  this->output_shift_ = output_shift;
  this->map_->updateMaxOccDist(max_occ_dist);
}

void
PointCloudScanner::setMapFactors(double off_map_factor,
                         double non_free_space_factor,
                         double non_free_space_radius)
{
  this->off_map_factor_ = off_map_factor;
  this->non_free_space_factor_ = non_free_space_factor;
  this->non_free_space_radius_ = non_free_space_radius;
}

void
PointCloudScanner::setPointCloudScannerToFootprintTF(tf::Transform point_cloud_scanner_to_footprint_tf)
{
  this->point_cloud_scanner_to_footprint_tf_ = point_cloud_scanner_to_footprint_tf;
}

// Update the filter based on the sensor model.  Returns true if the
// filter has been updated.
bool
PointCloudScanner::updateSensor(ParticleFilter *pf, SensorData *data)
{
  if (this->max_beams_ < 2)
    return false;
  // Apply the point cloud scanner sensor model
  pf->updateSensor((PFSensorModelFnPtr) applyModelToSampleSet, data);

  return true;
}

// Update a sample set based on the sensor model.
// Returns total weights of particles, or 0.0 on failure.
double
PointCloudScanner::applyModelToSampleSet(SensorData *data, PFSampleSet *set)
{
  PointCloudScanner *self;
  double rv = 0.0;
  int j;
  PFSample *sample;
  PFVector pose;
  int mi, mj;

  self = (PointCloudScanner*) data->sensor_;
  if(self->max_beams_ < 2)
      return 0.0;
  if(self->model_type_ == POINT_CLOUD_MODEL)
  {
    rv = calcPointCloudModel((PointCloudData*) data, set);
  }
  else if(self->model_type_ == POINT_CLOUD_MODEL_GOMPERTZ)
  {
    rv = calcPointCloudModelGompertz((PointCloudData*) data, set);
  }

  // Apply the any configured correction factors from map
  if (rv > 0.0)
  {
    // Re-calculate total
    rv = 0.0;
    for (j = 0; j < set->sample_count; j++)
    {
      sample = set->samples + j;
      pose = sample->pose;

      // Convert to map grid coords.
      self->map_->convertWorldToMap({pose.v[0], pose.v[1]}, &self->map_vec_);
      mi = self->map_vec_[0];
      mj = self->map_vec_[1];

      // Apply off map factor
      if(!self->map_->isValid({mi, mj}))
      {
        sample->weight *= self->off_map_factor_;
      }
      rv += sample->weight;
    }
  }
  return rv;
}

// Determine the probability for the given pose
double
PointCloudScanner::calcPointCloudModel(PointCloudData *data, PFSampleSet* set)
{
  PointCloudScanner *self;
  double total_weight = 0.0, p, z, pz;
  PFSample *sample;
  PFVector pose;

  self = (PointCloudScanner*) data->sensor_;
  const double MAX_OCC_DIST = self->map_->getMaxOccDist();
  double z_hit_denom = 2 * self->sigma_hit_ * self->sigma_hit_;
  double z_rand_mult = 1.0/MAX_OCC_DIST;

  for(int sample_index = 0; sample_index < set->sample_count; sample_index++)
  {
    sample = set->samples + sample_index;
    pose = sample->pose;
    p = 1.0;
    pcl::PointCloud<pcl::PointXYZ>::iterator it;
    pcl::PointCloud<pcl::PointXYZ> map_cloud;
    if(!getMapCloud(self, data, pose, map_cloud))
      return 0.0;
    for(it = map_cloud.begin(); it != map_cloud.end(); ++it)
    {
      self->map_->convertWorldToMap({it->x, it->y, it->z}, &self->map_vec_);
      z = self->map_->getOccDist(self->map_vec_[0], self->map_vec_[1], self->map_vec_[2]);
      pz = self->z_hit_ * exp(-(z*z) / z_hit_denom);
      pz += self->z_rand_ * z_rand_mult;
      assert(pz <= 1.0);
      assert(pz >= 0.0);
      p += pz*pz*pz;
    }
    sample->weight *= p;
    total_weight += sample->weight;
  }
  return total_weight;
}

double
PointCloudScanner::calcPointCloudModelGompertz(PointCloudData *data, PFSampleSet* set)
{
  PointCloudScanner *self;
  double total_weight = 0.0, p, z, pz, sum_pz;
  PFSample *sample;
  PFVector pose;
  self = (PointCloudScanner*) data->sensor_;
  const double MAX_OCC_DIST = self->map_->getMaxOccDist();
  double z_hit_denom = 2 * self->sigma_hit_ * self->sigma_hit_;
  for(int sample_index = 0; sample_index < set->sample_count; sample_index++)
  {
    sample = set->samples + sample_index;
    pose = sample->pose;
    pcl::PointCloud<pcl::PointXYZ>::iterator it;
    pcl::PointCloud<pcl::PointXYZ> map_cloud;
    if(!getMapCloud(self, data, pose, map_cloud))
      return 0.0;
    sum_pz = 0;
    int count = 0;
    for(it = map_cloud.begin(); it != map_cloud.end(); ++it)
    {
      self->map_->convertWorldToMap({it->x, it->y, it->z}, &self->map_vec_);
      z = self->map_->getOccDist(self->map_vec_[0], self->map_vec_[1], self->map_vec_[2]);
      pz = self->z_hit_ * exp(-(z*z) / z_hit_denom);
      pz += self->z_rand_;
      sum_pz += pz;
      count++;
    }
    p = sum_pz / count;
    p = self->applyGompertz(p);
    sample->weight *= p;
    total_weight += sample->weight;
  }
  return total_weight;
}

bool
PointCloudScanner::getMapCloud(PointCloudScanner *self, PointCloudData *data,
                               PFVector pose, pcl::PointCloud<pcl::PointXYZ>& map_cloud)
{
  pcl::PointCloud<pcl::PointXYZ> footprint_cloud;
  tf::Vector3 footprint_to_map_origin(pose.v[0], pose.v[1], 0.0);
  tf::Quaternion footprint_to_map_q(tf::Vector3(0.0, 0.0, 1.0), pose.v[2]);
  tf::Transform footprint_to_map_tf(footprint_to_map_q, footprint_to_map_origin);
  try
  {
    tf::Matrix3x3 m = self->point_cloud_scanner_to_footprint_tf_.getBasis();
    pcl_ros::transformPointCloud(data->points_, footprint_cloud, self->point_cloud_scanner_to_footprint_tf_);
    pcl_ros::transformPointCloud(footprint_cloud, map_cloud, footprint_to_map_tf);
  }
  catch(tf::TransformException& e)
  {
    ROS_ERROR("Failed to transform sensor point cloud to map point cloud.");
    return false;
  }
  return true;
}

double
PointCloudScanner::applyGompertz(double p)
{
  // shift and scale p
  p = p * this->input_scale_ + this->input_shift_;
  // apply gompertz
  p = this->gompertz_a_ * exp( -1.0 * this->gompertz_b_ * exp( -1.0 * this->gompertz_c_ * p ) );
  // shift output
  p += this->output_shift_;

  return p;
}

int
PointCloudScanner::getMaxBeams()
{
  return max_beams_;
}

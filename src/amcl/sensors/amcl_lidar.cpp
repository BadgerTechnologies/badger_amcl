/*
 *
 *       AMCL Octomap Class
 *        by Tyler Buchman
 *        2019
 *
 */

#include "amcl_lidar.h"
#include "ros/ros.h"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>


using namespace amcl;


AMCLLidar::AMCLLidar(size_t max_beams, OctoMap* map, double lidar_height) : AMCLSensor()
{
  this->time = 0.0;

  this->max_beams = max_beams;
  this->map = map;
  this->lidar_height = lidar_height;

  off_map_factor = 1.0;
  non_free_space_factor = 1.0;
  non_free_space_radius = 0.0;
}

AMCLLidar::~AMCLLidar()
{
  delete map;
}

void
AMCLLidar::SetModelLidarType(double z_hit, double z_rand, double sigma_hit, double max_occ_dist)
{
  this->model_type = LIDAR_MODEL;
  this->z_hit = z_hit;
  this->z_rand = z_rand;
  this->sigma_hit = sigma_hit;
  this->map->updateCSpace(max_occ_dist);
}

void
AMCLLidar::SetMapFactors(double off_map_factor,
                         double non_free_space_factor,
                         double non_free_space_radius)
{
  this->off_map_factor = off_map_factor;
  this->non_free_space_factor = non_free_space_factor;
  this->non_free_space_radius = non_free_space_radius;
}

void
AMCLLidar::SetLidarToFootprintTF(tf::Transform lidar_to_footprint_tf)
{
  this->lidar_to_footprint_tf = lidar_to_footprint_tf;
}

// Update the filter based on the sensor model.  Returns true if the
// filter has been updated.
bool
AMCLLidar::UpdateSensor(ParticleFilter *pf, AMCLSensorData *data)
{
  if (this->max_beams < 2)
    return false;
  // Apply the lidar sensor model
  pf->update_sensor((pf_sensor_model_fn_t) ApplyModelToSampleSet, data);

  return true;
}

// Update a sample set based on the sensor model.
// Returns total weights of particles, or 0.0 on failure.
double
AMCLLidar::ApplyModelToSampleSet(AMCLSensorData *data, pf_sample_set_t *set)
{
  AMCLLidar *self;
  double rv = 0.0;
  int j;
  pf_sample_t *sample;
  PFVector pose;
  int mi, mj;

  self = (AMCLLidar*) data->sensor;
  if(self->max_beams < 2)
      return 0.0;
  if(self->model_type == LIDAR_MODEL)
  {
    rv = LidarModel((AMCLLidarData*) data, set);
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
      std::vector<int> m_vec;
      m_vec = self->map->convertWorldToMap({pose.v[0], pose.v[1]});
      mi = m_vec[0];
      mj = m_vec[1];

      // Apply off map factor
      if(!self->map->isValid({mi, mj}))
      {
        sample->weight *= self->off_map_factor;
      }
      /*
      // Apply non free space factor
      else
      {
        double dist = self->map->getOccDist(mi, mj);

        if(dist == 0.0)
        {
          sample->weight *= self->non_free_space_factor;
        }
        // Interpolate non free space factor based on radius
        else if(dist < self->non_free_space_radius)
        {
          double delta_d = dist / self->non_free_space_radius;
          double f = self->non_free_space_factor;
          f += delta_d * (1.0 - self->non_free_space_factor);
          sample->weight *= f;
        }
      }
      */
      rv += sample->weight;
    }
  }
  return rv;
}

// Determine the probability for the given pose
double
AMCLLidar::LidarModel(AMCLLidarData *data, pf_sample_set_t* set)
{
  AMCLLidar *self;
  double total_weight = 0.0, p, z, pz;
  pf_sample_t *sample;
  PFVector pose;
  int count = 0;

  self = (AMCLLidar*) data->sensor;
  const double MAX_OCC_DIST = self->map->getMaxOccDist();
  double z_hit_denom = 2 * self->sigma_hit * self->sigma_hit;
  double z_rand_mult = 1.0/MAX_OCC_DIST;

  int min_value = 100000;
  int max_value = 0;
  int count_max = 0;
  int count_min = 0;
  double max_weight = 0.0;

  for(int sample_index = 0; sample_index < set->sample_count; sample_index++)
  {
    sample = set->samples + sample_index;
    pose = sample->pose;
    p = 1.0;
    pcl::PointCloud<pcl::PointXYZ>::iterator it;

    count_max = 0;
    count_min = 0;

    pcl::PointCloud<pcl::PointXYZ> footprint_cloud, map_cloud;

    tf::Vector3 footprint_to_map_origin(pose.v[0], pose.v[1], 0.0);
    tf::Quaternion footprint_to_map_q(tf::Vector3(0.0, 0.0, 1.0), pose.v[2]);
    tf::Transform footprint_to_map_tf(footprint_to_map_q, footprint_to_map_origin);

    try
    {
      it = data->points.begin();
      tf::Matrix3x3 m = self->lidar_to_footprint_tf.getBasis();
      pcl_ros::transformPointCloud(data->points, footprint_cloud, self->lidar_to_footprint_tf);
      it = footprint_cloud.begin();
      pcl_ros::transformPointCloud(footprint_cloud, map_cloud, footprint_to_map_tf);
      it = map_cloud.begin();
    }
    catch(tf::TransformException& e)
    {
      ROS_ERROR("Failed to transform lidar point cloud to map point cloud.");
      return 0.0;
    }
    int count = 0;
    bool publish = (sample_index % 1000 == 2);
    for(it = map_cloud.begin(); it != map_cloud.end(); ++it)
    {
      std::vector<int> map_coords = self->map->convertWorldToMap({it->x, it->y, it->z});
      z = self->map->getOccDist(map_coords[0], map_coords[1], map_coords[2], (++count % 100 == 0 and false));
      pz = self->z_hit * exp(-(z*z) / z_hit_denom);
      pz += self->z_rand * z_rand_mult;
      assert(pz <= 1.0);
      assert(pz >= 0.0);
      double p_before = p;
      p += pz*pz*pz;

      if(z != 0.36)
        count_max ++;
      if(z == 0.36)
        count_min ++;
      if(publish and false)
      {
        ROS_INFO("z_hit: %f, z_hit_denom: %f", self->z_hit, z_hit_denom);
        ROS_INFO("z: %f, -(z*z): %f, e: %f", z, -(z*z), exp(-(z*z) / z_hit_denom));
        ROS_INFO("z_rand: %f, z_rand_mult: %f", self->z_rand, z_rand_mult);
        ROS_INFO("pz1: %f, pz2: %f, pz: %f, p: %f", self->z_hit * exp(-(z*z) / z_hit_denom), self->z_rand * z_rand_mult, pz, p);
      }
    }
    double weight_before = sample->weight;
    sample->weight *= p;
    total_weight += sample->weight;

    count_max *= 10000;
    count_max += 1000 - count_min;
    max_value = std::max(max_value, count_max);
    min_value = std::min(min_value, count_min);
    max_weight = std::max(max_weight, sample->weight);

    if(publish)
    {
      ROS_INFO("sample weight before: %f", weight_before);
      ROS_INFO("sample weight after: %f", sample->weight);
    }
    //ROS_INFO("pose: %f, %f, weight: %f", pose.v[0], pose.v[1], sample->weight);
    if(false)
    {
      for(z = 0.0; z < 1.0; z += 0.01)
      {
        ROS_INFO("z: %f, -(z*z): %f, inside: %f, pz1: %f, pz2: %f", z, -(z*z), -(z*z) / z_hit_denom, self->z_hit * exp(-(z*z) / z_hit_denom), self->z_rand * z_rand_mult);
      }
    }
  }
  return total_weight;
}

int
AMCLLidar::getMaxBeams()
{
  return max_beams;
}

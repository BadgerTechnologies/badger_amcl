/*
 *
 *       AMCL Octomap Class
 *        by Tyler Buchman
 *        2019
 *
 */

#ifndef AMCL_LIDAR_H
#define AMCL_LIDAR_H

#include "amcl_sensor.h"
#include "../map/map.h"
#include "../map/octomap.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "tf/tf.h"

namespace amcl
{


typedef enum
{
  LIDAR_MODEL,
} lidar_model_t;


class AMCLLidarData : public AMCLSensorData
{
  public:
    AMCLLidarData () {};
    virtual ~AMCLLidarData() {};
    std::string frame_id;
    pcl::PointCloud<pcl::PointXYZ> points;
};


class AMCLLidar : public AMCLSensor
{
  public:
    AMCLLidar(size_t max_beams, OctoMap* map, double lidar_height);
    ~AMCLLidar();

    void SetModelLidarType(double z_hit, double z_rand,
                           double sigma_hit, double max_occ_dist);

    // Update the filter based on the sensor model.  Returns true if the
    // filter has been updated.
    bool UpdateSensor(ParticleFilter *pf, AMCLSensorData *data);

    // Update a sample set based on the sensor model.
    // Returns total weights of particles, or 0.0 on failure.
    static double ApplyModelToSampleSet(AMCLSensorData *data, pf_sample_set_t *set);

    void SetMapFactors(double off_map_factor,
                       double non_free_space_factor,
                       double non_free_space_radius);

    // Set the lidar's pose after construction
    void SetLidarPose(PFVector& lidar_pose)
    {this->lidar_pose = lidar_pose;}

    void SetLidarToFootprintTF(tf::Transform lidarToFootprintTF);

    int getMaxBeams();

  private:
    // Determine the probability for the given pose
    static double LidarModel(AMCLLidarData *data, pf_sample_set_t *set);

    OctoMap *map;
    PFVector lidar_pose;
    lidar_model_t model_type;
    // Current data timestamp
    double time;
    double off_map_factor;
    double non_free_space_factor;
    double non_free_space_radius;
    // Mixture params for the components of the model; must sum to 1
    double z_hit;
    double z_short;
    double z_max;
    double z_rand;
    // Stddev of Gaussian model for lidar hits.
    double sigma_hit;
    // Max beams to consider
    int max_beams;
    double lidar_height;

    tf::Transform lidar_to_footprint_tf;
};

}

#endif

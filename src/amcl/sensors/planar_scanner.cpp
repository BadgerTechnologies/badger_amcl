/*
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

#include "sensors/planar_scanner.h"

#include <cmath>
#include <cstdlib>
#include <functional>

#include <angles/angles.h>

namespace badger_amcl
{

////////////////////////////////////////////////////////////////////////////////
// Default constructor
PlanarScanner::PlanarScanner()
    : Sensor(),
      max_beams_(0),
      max_samples_(0),
      max_obs_(0)
{
  off_map_factor_ = 1.0;
  non_free_space_factor_ = 1.0;
  non_free_space_radius_ = 0.0;
  pixel_.resize(2);
  point_.resize(2);
}

void PlanarScanner::init(int max_beams, std::shared_ptr<OccupancyMap> map,
    double z_hit, double z_rand, double sigma_hit, double max_distance_to_object,
    double gompertz_a, double gompertz_b, double gompertz_c,
    double input_shift, double input_scale, double output_shift)
{
  max_beams_ = max_beams;
  map_ = map;
  z_hit_ = z_hit;
  z_rand_ = z_rand;
  sigma_hit_ = sigma_hit;

  gompertz_a_ = gompertz_a;
  gompertz_b_ = gompertz_b;
  gompertz_c_ = gompertz_c;
  input_shift_ = input_shift;
  input_scale_ = input_scale;
  output_shift_ = output_shift;
  map_->updateDistancesLUT(max_distance_to_object);
}

void PlanarScanner::setMapFactors(double off_map_factor, double non_free_space_factor,
                                  double non_free_space_radius)
{
  off_map_factor_ = off_map_factor;
  non_free_space_factor_ = non_free_space_factor;
  non_free_space_radius_ = non_free_space_radius;
}

////////////////////////////////////////////////////////////////////////////////
// Apply the planar sensor model
bool PlanarScanner::updateSensor(std::shared_ptr<ParticleFilter> pf,
                                 std::shared_ptr<SensorData> data)
{
  if (max_beams_ < 2)
    return false;

  // Apply the planar sensor model
  std::function<double(std::shared_ptr<SensorData>, std::shared_ptr<PFSampleSet>)> sensor_fn = (
          std::bind(&PlanarScanner::applyModelToSampleSet, this,
                    std::placeholders::_1, std::placeholders::_2));
  pf->updateSensor(sensor_fn, data);
  return true;
}

////////////////////////////////////////////////////////////////////////////////
// Apply the planar sensor model to a sample set
double PlanarScanner::applyModelToSampleSet(std::shared_ptr<SensorData> data,
                                            std::shared_ptr<PFSampleSet> set)
{
  if (max_beams_ < 2)
    return 0.0;

  // Apply the planar sensor model
  double total_weight = calcLikelihoodFieldModelGompertz(std::dynamic_pointer_cast<PlanarData>(data), set);

  // Apply the any configured correction factors from map
  if (total_weight > 0.0)
  {
    total_weight = applyOffMapFactor(set);
  }
  return total_weight;
}

void PlanarScanner::setPlanarScannerPose(const Eigen::Vector3d& scanner_pose)
{
  planar_scanner_pose_ = scanner_pose;
}

double PlanarScanner::calcLikelihoodFieldModelGompertz(std::shared_ptr<PlanarData> data,
                                                       std::shared_ptr<PFSampleSet> set)
{
  int i, j, step;
  double z, pz;
  double p;
  double obs_range, obs_bearing;
  double total_weight;
  PFSample* sample;
  Eigen::Vector3d pose;
  Eigen::Vector3d hit;

  total_weight = 0.0;

  // Compute the sample weights
  for (j = 0; j < set->sample_count; j++)
  {
    sample = &(set->samples[j]);
    pose = sample->pose;

    // Take account of the planar scanner pose relative to the robot
    pose = coordAdd(planar_scanner_pose_, pose);

    // Pre-compute a couple of things
    double z_hit_denom = 2 * sigma_hit_ * sigma_hit_;

    step = (data->range_count_ - 1) / (max_beams_ - 1);

    // Step size must be at least 1
    if (step < 1)
      step = 1;

    int valid_beams = 0;
    double sum_pz = 0.0;
    for (i = 0; i < data->range_count_; i += step)
    {
      obs_range = data->ranges_[i];
      obs_bearing = data->angles_[i];

      // This model ignores max range readings
      if (obs_range >= data->range_max_)
        continue;

      // Check for NaN
      if (obs_range != obs_range)
        continue;

      valid_beams++;
      pz = 0.0;

      // Compute the endpoint of the beam
      hit[0] = pose[0] + obs_range * std::cos(pose[2] + obs_bearing);
      hit[1] = pose[1] + obs_range * std::sin(pose[2] + obs_bearing);

      // Convert to map grid coords.
      point_[0] = hit[0];
      point_[1] = hit[1];
      map_->rasterize(point_, &pixel_);
      // Part 1: Get distance from the hit to closest obstacle.
      // Off-map penalized as max distance
      if (!map_->isValid(pixel_))
        z = map_->getMaxDistanceToObject();
      else
        z = map_->getDistanceToObject(pixel_[0], pixel_[1]);
      // Gaussian model
      pz += z_hit_ * std::exp(-(z * z) / z_hit_denom);
      // Part 2: random measurements
      pz += z_rand_;

      sum_pz += pz;
    }

    if (valid_beams > 0)
    {
      p = sum_pz / valid_beams;
      p = applyGompertz(p);
    }
    else
    {
      // Hmm. No valid beams. Don't change the weight.
      p = 1.0;
    }

    sample->weight *= p;
    total_weight += sample->weight;
  }

  return total_weight;
}

double PlanarScanner::applyGompertz(double p)
{
  // shift and scale p
  p = p * input_scale_ + input_shift_;
  // apply gompertz
  p = gompertz_a_ * std::exp(-1.0 * gompertz_b_ * std::exp(-1.0 * gompertz_c_ * p));
  // shift output
  p += output_shift_;

  return p;
}

double PlanarScanner::applyOffMapFactor(std::shared_ptr<PFSampleSet> set)
{
  double total_weight = 0.0;
  PFSample* sample;
  Eigen::Vector3d pose;
  for (int j = 0; j < set->sample_count; j++)
  {
    sample = &(set->samples[j]);
    pose = sample->pose;

    // Convert to map grid coords.
    point_[0] = pose[0];
    point_[1] = pose[1];
    map_->rasterize(point_, &pixel_);

    // Apply off map factor
    if (!map_->isValid(pixel_))
    {
      sample->weight *= off_map_factor_;
    }
    // Apply non free space factor
    else if (map_->getCellState(pixel_[0], pixel_[1]) != MapCellState::CELL_FREE)
    {
      sample->weight *= non_free_space_factor_;
    }
    // Interpolate non free space factor based on radius
    else
    {
      double distance = map_->getDistanceToObject(pixel_[0], pixel_[1]);
      if (distance < non_free_space_radius_)
      {
        double delta_d = map_->getDistanceToObject(pixel_[0], pixel_[1]) / non_free_space_radius_;
        double f = non_free_space_factor_;
        f += delta_d * (1.0 - non_free_space_factor_);
        sample->weight *= f;
      }
    }
    total_weight += sample->weight;
  }
  return total_weight;
}

void PlanarScanner::clearTempData(int new_max_samples, int new_max_obs)
{
  max_obs_ = new_max_obs;
  max_samples_ = fmax(max_samples_, new_max_samples);
  temp_obs_.clear();
  temp_obs_.resize(max_samples_, std::vector<double>(max_obs_, 0.0));
}

// Transform from local to global coords (a + b)
Eigen::Vector3d PlanarScanner::coordAdd(const Eigen::Vector3d& a, const Eigen::Vector3d& b)
{
  Eigen::Vector3d c;
  c[0] = b[0] + a[0] * std::cos(b[2]) - a[1] * std::sin(b[2]);
  c[1] = b[1] + a[0] * std::sin(b[2]) + a[1] * std::cos(b[2]);
  c[2] = b[2] + a[2];
  c[2] = angles::normalize_angle(c[2]);
  return c;
}

}  // namespace badger_amcl

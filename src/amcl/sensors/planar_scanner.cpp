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
#include <ros/assert.h>
#include <ros/console.h>

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
  map_vec_.resize(2);
  world_vec_.resize(2);
}

void PlanarScanner::init(int max_beams, std::shared_ptr<OccupancyMap> map)
{
  max_beams_ = max_beams;
  map_ = map;
}

void PlanarScanner::setModelBeam(double z_hit, double z_short, double z_max, double z_rand,
                                 double sigma_hit, double lambda_short)
{
  model_type_ = PLANAR_MODEL_BEAM;
  z_hit_ = z_hit;
  z_short_ = z_short;
  z_max_ = z_max;
  z_rand_ = z_rand;
  sigma_hit_ = sigma_hit;
  lambda_short_ = lambda_short;
}

void PlanarScanner::setModelLikelihoodField(double z_hit, double z_rand, double sigma_hit,
                                            double max_distance_to_object)
{
  model_type_ = PLANAR_MODEL_LIKELIHOOD_FIELD;
  z_hit_ = z_hit;
  z_rand_ = z_rand;
  sigma_hit_ = sigma_hit;
  map_->updateDistancesLUT(max_distance_to_object);
}

void PlanarScanner::setModelLikelihoodFieldProb(double z_hit, double z_rand, double sigma_hit,
                                                double max_distance_to_object, bool do_beamskip,
                                                double beam_skip_distance,
                                                double beam_skip_threshold,
                                                double beam_skip_error_threshold)
{
  model_type_ = PLANAR_MODEL_LIKELIHOOD_FIELD_PROB;
  z_hit_ = z_hit;
  z_rand_ = z_rand;
  sigma_hit_ = sigma_hit;
  do_beamskip_ = do_beamskip;
  beam_skip_distance_ = beam_skip_distance;
  beam_skip_threshold_ = beam_skip_threshold;
  beam_skip_error_threshold_ = beam_skip_error_threshold;
  map_->updateDistancesLUT(max_distance_to_object);
}

void PlanarScanner::setModelLikelihoodFieldGompertz(double z_hit, double z_rand, double sigma_hit,
                                                    double max_distance_to_object, double gompertz_a,
                                                    double gompertz_b, double gompertz_c,
                                                    double input_shift, double input_scale,
                                                    double output_shift)
{
  ROS_INFO("Initializing model likelihood field gompertz");
  model_type_ = PLANAR_MODEL_LIKELIHOOD_FIELD_GOMPERTZ;
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

  double rv = 0.0;
  // Apply the planar sensor model
  if (model_type_ == PLANAR_MODEL_BEAM)
    rv = calcBeamModel(std::dynamic_pointer_cast<PlanarData>(data), set);
  else if (model_type_ == PLANAR_MODEL_LIKELIHOOD_FIELD)
    rv = calcLikelihoodFieldModel(std::dynamic_pointer_cast<PlanarData>(data), set);
  else if (model_type_ == PLANAR_MODEL_LIKELIHOOD_FIELD_PROB)
    rv = calcLikelihoodFieldModelProb(std::dynamic_pointer_cast<PlanarData>(data), set);
  else if (model_type_ == PLANAR_MODEL_LIKELIHOOD_FIELD_GOMPERTZ)
    rv = calcLikelihoodFieldModelGompertz(std::dynamic_pointer_cast<PlanarData>(data), set);

  // Apply the any configured correction factors from map
  if (rv > 0.0)
  {
    rv = recalcWeight(set);
  }
  return rv;
}

////////////////////////////////////////////////////////////////////////////////
// Determine the probability for the given pose
double PlanarScanner::calcBeamModel(std::shared_ptr<PlanarData> data,
                                    std::shared_ptr<PFSampleSet> set)
{
  int i, j, step;
  double z, pz;
  double p;
  double map_range;
  double obs_range, obs_bearing;
  double total_weight;
  PFSample* sample;
  Eigen::Vector3d pose;

  total_weight = 0.0;

  // Compute the sample weights
  for (j = 0; j < set->sample_count; j++)
  {
    sample = &(set->samples[j]);
    pose = sample->pose;

    // Take account of the planar scanner pose relative to the robot
    pose = coordAdd(planar_scanner_pose_, pose);

    p = 1.0;

    step = (data->range_count_ - 1) / (max_beams_ - 1);
    for (i = 0; i < data->range_count_; i += step)
    {
      obs_range = data->ranges_[i];
      obs_bearing = data->angles_[i];

      // Compute the range according to the map
      map_range = map_->calcRange(pose[0], pose[1], pose[2] + obs_bearing, data->range_max_);
      pz = 0.0;

      // Part 1: good, but noisy, hit
      z = obs_range - map_range;
      pz += z_hit_ * std::exp(-(z * z) / (2 * sigma_hit_ * sigma_hit_));

      // Part 2: short reading from unexpected obstacle (e.g., a person)
      if (z < 0)
        pz += z_short_ * lambda_short_ * std::exp(-lambda_short_ * obs_range);

      // Part 3: Failure to detect obstacle, reported as max-range
      if (obs_range == data->range_max_)
        pz += z_max_ * 1.0;

      // Part 4: Random measurements
      if (obs_range < data->range_max_)
        pz += z_rand_ * 1.0 / data->range_max_;

      // TODO(anyone): outlier rejection for short readings

      ROS_ASSERT(pz <= 1.0);
      ROS_ASSERT(pz >= 0.0);
      //      p *= pz;
      // here we have an ad-hoc weighting scheme for combining beam probs
      // works well, though...
      p += pz * pz * pz;
    }

    sample->weight *= p;
    total_weight += sample->weight;
  }

  return total_weight;
}

double PlanarScanner::calcLikelihoodFieldModel(std::shared_ptr<PlanarData> data,
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

    p = 1.0;

    // Pre-compute a couple of things
    double z_hit_denom = 2 * sigma_hit_ * sigma_hit_;
    double z_rand_mult = 1.0 / data->range_max_;

    step = (data->range_count_ - 1) / (max_beams_ - 1);

    // Step size must be at least 1
    if (step < 1)
      step = 1;

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

      pz = 0.0;

      // Compute the endpoint of the beam
      hit[0] = pose[0] + obs_range * std::cos(pose[2] + obs_bearing);
      hit[1] = pose[1] + obs_range * std::sin(pose[2] + obs_bearing);

      // Convert to map_ grid coords.
      world_vec_[0] = hit[0];
      world_vec_[1] = hit[1];
      map_->convertWorldToMap(world_vec_, &map_vec_);

      // Part 1: Get distance from the hit to closest obstacle.
      // Off-map penalized as max distance
      if (!map_->isValid(map_vec_))
        z = map_->getMaxDistanceToObject();
      else
        z = map_->getDistanceToObject(map_vec_[0], map_vec_[1]);
      // Gaussian model
      // NOTE: this should have a normalization of 1/(sqrt(2pi)*sigma)
      pz += z_hit_ * std::exp(-(z * z) / z_hit_denom);
      // Part 2: random measurements
      pz += z_rand_ * z_rand_mult;

      // TODO(anyone): outlier rejection for short readings

      ROS_ASSERT(pz <= 1.0);
      ROS_ASSERT(pz >= 0.0);
      //      p *= pz;
      // here we have an ad-hoc weighting scheme for combining beam probs
      // works well, though...
      // TODO(anyone): investigate schemes for combining beam probs
      p += pz * pz * pz;
    }

    sample->weight *= p;
    total_weight += sample->weight;
  }

  return total_weight;
}

double PlanarScanner::calcLikelihoodFieldModelProb(std::shared_ptr<PlanarData> data,
                                                   std::shared_ptr<PFSampleSet> set)
{
  int i, j, step;
  double z, pz;
  double log_p;
  double obs_range, obs_bearing;
  double total_weight;
  PFSample* sample;
  Eigen::Vector3d pose;
  Eigen::Vector3d hit;

  total_weight = 0.0;

  step = std::ceil((data->range_count_) / static_cast<double>(max_beams_));

  // Step size must be at least 1
  if (step < 1)
    step = 1;

  // Pre-compute a couple of things
  double z_hit_denom = 2 * sigma_hit_ * sigma_hit_;
  double z_rand_mult = 1.0 / data->range_max_;

  double max_distance_to_object = map_->getMaxDistanceToObject();
  double max_dist_prob = std::exp(-(max_distance_to_object * max_distance_to_object) / z_hit_denom);

  // Beam skipping - ignores beams for which a majoirty of particles do not agree with the map
  // prevents correct particles from getting down weighted because of unexpected obstacles
  // such as humans

  bool do_beamskip = do_beamskip_;
  double beam_skip_distance = beam_skip_distance_;
  double beam_skip_threshold = beam_skip_threshold_;

  // we only do beam skipping if the filter has converged
  if (do_beamskip && !set->converged)
  {
    do_beamskip = false;
  }

  // we need a count the no of particles for which the beam agreed with the map
  std::vector<int> obs_count(max_beams_);

  // we also need a mask of which observations to integrate
  // (to decide which beams to integrate to all particles)
  std::vector<bool> obs_mask(max_beams_);

  int beam_ind = 0;

  // clear_temp indicates if we need to clear the temp data structure needed to do beamskipping
  bool clear_temp = false;

  if (do_beamskip)
  {
    if (max_obs_ < max_beams_)
    {
      clear_temp = true;
    }

    if (max_samples_ < set->sample_count)
    {
      clear_temp = true;
    }

    if (clear_temp)
    {
      clearTempData(set->sample_count, max_beams_);
      ROS_DEBUG_STREAM("Clearing temp weights " << max_samples_ << " - " << max_obs_);
    }
  }

  // Compute the sample weights
  for (j = 0; j < set->sample_count; j++)
  {
    sample = &(set->samples[j]);
    pose = sample->pose;

    // Take account of the planar scanner pose relative to the robot
    pose = coordAdd(planar_scanner_pose_, pose);

    log_p = 0;

    beam_ind = 0;

    for (i = 0; i < data->range_count_; i += step, beam_ind++)
    {
      obs_range = data->ranges_[i];
      obs_bearing = data->angles_[i];

      // This model ignores max range readings
      if (obs_range >= data->range_max_)
      {
        continue;
      }

      // Check for NaN
      if (obs_range != obs_range)
      {
        continue;
      }

      pz = 0.0;

      // Compute the endpoint of the beam
      hit[0] = pose[0] + obs_range * std::cos(pose[2] + obs_bearing);
      hit[1] = pose[1] + obs_range * std::sin(pose[2] + obs_bearing);

      // Convert to map grid coords.
      world_vec_[0] = hit[0];
      world_vec_[1] = hit[1];
      map_->convertWorldToMap(world_vec_, &map_vec_);

      // Part 1: Get distance from the hit to closest obstacle.
      // Off-map penalized as max distance

      if (!map_->isValid(map_vec_))
      {
        pz += z_hit_ * max_dist_prob;
      }
      else
      {
        z = map_->getDistanceToObject(map_vec_[0], map_vec_[1]);
        if (z < beam_skip_distance)
        {
          obs_count[beam_ind] += 1;
        }
        pz += z_hit_ * std::exp(-(z * z) / z_hit_denom);
      }

      // Gaussian model
      // NOTE: this should have a normalization of 1/(sqrt(2pi)*sigma)

      // Part 2: random measurements
      pz += z_rand_ * z_rand_mult;

      ROS_ASSERT(pz <= 1.0);
      ROS_ASSERT(pz >= 0.0);

      // TODO(anyone): outlier rejection for short readings

      if (!do_beamskip)
      {
        log_p += std::log(pz);
      }
      else
      {
        temp_obs_[j][beam_ind] = pz;
      }
    }
    if (!do_beamskip)
    {
      sample->weight *= std::exp(log_p);
      total_weight += sample->weight;
    }
  }

  if (do_beamskip)
  {
    int skipped_beam_count = 0;
    for (beam_ind = 0; beam_ind < max_beams_; beam_ind++)
    {
      if ((obs_count[beam_ind] / static_cast<double>(set->sample_count)) > beam_skip_threshold)
      {
        obs_mask[beam_ind] = true;
      }
      else
      {
        obs_mask[beam_ind] = false;
        skipped_beam_count++;
      }
    }

    // we check if there is at least a critical number of beams that agreed with the map
    // otherwise it probably indicates that the filter converged to a wrong solution
    // if that's the case we integrate all the beams and hope the filter might converge to
    // the right solution
    bool error = false;

    if (skipped_beam_count >= (beam_ind * beam_skip_error_threshold_))
    {
      ROS_DEBUG("Over %f%% of the observations were not in the map - pf may have converged to "
                "wrong pose - integrating all observations",
                (100 * beam_skip_error_threshold_));
      error = true;
    }

    for (j = 0; j < set->sample_count; j++)
    {
      sample = &(set->samples[j]);
      pose = sample->pose;

      log_p = 0;

      for (beam_ind = 0; beam_ind < max_beams_; beam_ind++)
      {
        if (error || obs_mask[beam_ind])
        {
          log_p += std::log(temp_obs_[j][beam_ind]);
        }
      }

      sample->weight *= std::exp(log_p);
      total_weight += sample->weight;
    }
  }

  return total_weight;
}

void PlanarScanner::setPlanarScannerPose(const Eigen::Vector3d& scanner_pose)
{
  planar_scanner_pose_ = scanner_pose;
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
      world_vec_[0] = hit[0];
      world_vec_[1] = hit[1];
      map_->convertWorldToMap(world_vec_, &map_vec_);
      // Part 1: Get distance from the hit to closest obstacle.
      // Off-map penalized as max distance
      if (!map_->isValid(map_vec_))
        z = map_->getMaxDistanceToObject();
      else
        z = map_->getDistanceToObject(map_vec_[0], map_vec_[1]);
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

double PlanarScanner::recalcWeight(std::shared_ptr<PFSampleSet> set)
{
  double rv = 0.0;
  PFSample* sample;
  Eigen::Vector3d pose;
  for (int j = 0; j < set->sample_count; j++)
  {
    sample = &(set->samples[j]);
    pose = sample->pose;

    // Convert to map grid coords.
    world_vec_[0] = pose[0];
    world_vec_[1] = pose[1];
    map_->convertWorldToMap(world_vec_, &map_vec_);

    // Apply off map factor
    if (!map_->isValid(map_vec_))
    {
      sample->weight *= off_map_factor_;
    }
    // Apply non free space factor
    else if (map_->getCellState(map_vec_[0], map_vec_[1]) != MapCellState::CELL_FREE)
    {
      sample->weight *= non_free_space_factor_;
    }
    // Interpolate non free space factor based on radius
    else
    {
      double distance = map_->getDistanceToObject(map_vec_[0], map_vec_[1]);
      if (distance < non_free_space_radius_)
      {
        double delta_d = map_->getDistanceToObject(map_vec_[0], map_vec_[1]) / non_free_space_radius_;
        double f = non_free_space_factor_;
        f += delta_d * (1.0 - non_free_space_factor_);
        sample->weight *= f;
      }
    }
    rv += sample->weight;
  }
  return rv;
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

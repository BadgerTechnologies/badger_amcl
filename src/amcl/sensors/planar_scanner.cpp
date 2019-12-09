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
///////////////////////////////////////////////////////////////////////////
//
// Desc: AMCL planar scanner routines
// Author: Andrew Howard
// Maintainer: Tyler Buchman (tyler_buchman@jabil.com)
//
///////////////////////////////////////////////////////////////////////////

#include <sys/types.h> // required by Darwin
#include <math.h>
#include <stdlib.h>
#include <assert.h>
#include <unistd.h>
#include "ros/ros.h"

#include "planar_scanner.h"

using namespace amcl;

////////////////////////////////////////////////////////////////////////////////
// Default constructor
PlanarScanner::PlanarScanner(size_t max_beams, OccupancyMap* map) : Sensor(),
                             max_samples_(0), max_obs_(0), temp_obs_(NULL)
{
  this->max_beams_ = max_beams;
  this->map_ = map;

  this->off_map_factor_ = 1.0;
  this->non_free_space_factor_ = 1.0;
  this->non_free_space_radius_ = 0.0;

  if(this->map_vec_.size() != 2)
    this->map_vec_.reserve(2);
}

PlanarScanner::~PlanarScanner()
{
  if(temp_obs_){
	for(int k=0; k < max_samples_; k++){
	  delete [] temp_obs_[k];
	}
	delete []temp_obs_; 
  }
}

void 
PlanarScanner::setModelBeam(double z_hit,
                            double z_short,
                            double z_max,
                            double z_rand,
                            double sigma_hit,
                            double lambda_short)
{
  this->model_type_ = PLANAR_MODEL_BEAM;
  this->z_hit_ = z_hit;
  this->z_short_ = z_short;
  this->z_max_ = z_max;
  this->z_rand_ = z_rand;
  this->sigma_hit_ = sigma_hit;
  this->lambda_short_ = lambda_short;
}

void 
PlanarScanner::setModelLikelihoodField(double z_hit,
                                       double z_rand,
                                       double sigma_hit,
                                       double max_occ_dist)
{
  this->model_type_ = PLANAR_MODEL_LIKELIHOOD_FIELD;
  this->z_hit_ = z_hit;
  this->z_rand_ = z_rand;
  this->sigma_hit_ = sigma_hit;
  this->map_->updateCSpace(max_occ_dist);
}

void 
PlanarScanner::setModelLikelihoodFieldProb(double z_hit,
                                           double z_rand,
				                           double sigma_hit,
				                           double max_occ_dist,
				                           bool do_beamskip,
				                           double beam_skip_distance,
				                           double beam_skip_threshold, 
				                           double beam_skip_error_threshold)
{
  this->model_type_ = PLANAR_MODEL_LIKELIHOOD_FIELD_PROB;
  this->z_hit_ = z_hit;
  this->z_rand_ = z_rand;
  this->sigma_hit_ = sigma_hit;
  this->do_beamskip_ = do_beamskip;
  this->beam_skip_distance_ = beam_skip_distance;
  this->beam_skip_threshold_ = beam_skip_threshold;
  this->beam_skip_error_threshold_ = beam_skip_error_threshold;
  this->map_->updateCSpace(max_occ_dist);
}

void
PlanarScanner::setModelLikelihoodFieldGompertz(double z_hit,
                                               double z_rand,
                                               double sigma_hit,
                                               double max_occ_dist,
                                               double gompertz_a,
                                               double gompertz_b,
                                               double gompertz_c,
                                               double input_shift,
                                               double input_scale,
                                               double output_shift)
{
  ROS_INFO("Initializing model likelihood field gompertz");
  this->model_type_ = PLANAR_MODEL_LIKELIHOOD_FIELD_GOMPERTZ;
  this->z_hit_ = z_hit;
  this->z_rand_ = z_rand;
  this->sigma_hit_ = sigma_hit;

  this->gompertz_a_ = gompertz_a;
  this->gompertz_b_ = gompertz_b;
  this->gompertz_c_ = gompertz_c;
  this->input_shift_ = input_shift;
  this->input_scale_ = input_scale;
  this->output_shift_ = output_shift;
  this->map_->updateCSpace(max_occ_dist);
}

void
PlanarScanner::setMapFactors(double off_map_factor, double non_free_space_factor,
                             double non_free_space_radius)
{
  this->off_map_factor_ = off_map_factor;
  this->non_free_space_factor_ = non_free_space_factor;
  this->non_free_space_radius_ = non_free_space_radius;
}

////////////////////////////////////////////////////////////////////////////////
// Apply the planar sensor model
bool
PlanarScanner::updateSensor(ParticleFilter *pf, SensorData *data)
{
  if (this->max_beams_ < 2)
    return false;

  // Apply the planar sensor model
  pf->updateSensor((PFSensorModelFnPtr) applyModelToSampleSet, data);

  return true;
}

////////////////////////////////////////////////////////////////////////////////
// Apply the planar sensor model to a sample set
double
PlanarScanner::applyModelToSampleSet(SensorData *data, PFSampleSet *set)
{
  PlanarScanner *self;
  double rv = 0.0;
  int j;
  PFSample *sample;
  PFVector pose;
  int mi, mj;

  self = (PlanarScanner*) data->sensor_;

  if (self->max_beams_ < 2)
    return 0.0;

  // Apply the planar sensor model
  if(self->model_type_ == PLANAR_MODEL_BEAM)
    rv = calcBeamModel((PlanarData*)data, set);
  else if(self->model_type_ == PLANAR_MODEL_LIKELIHOOD_FIELD)
    rv = calcLikelihoodFieldModel((PlanarData*)data, set);
  else if(self->model_type_ == PLANAR_MODEL_LIKELIHOOD_FIELD_PROB)
    rv = calcLikelihoodFieldModelProb((PlanarData*)data, set);
  else if(self->model_type_ == PLANAR_MODEL_LIKELIHOOD_FIELD_GOMPERTZ)
    rv = calcLikelihoodFieldModelGompertz((PlanarData*)data, set);

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
      // Apply non free space factor
      else if(self->map_->getOccState(mi, mj) != -1)
      {
        sample->weight *= self->non_free_space_factor_;
      }
      // Interpolate non free space factor based on radius
      else if(self->map_->getOccDist(mi, mj) < self->non_free_space_radius_)
      {
        double delta_d = self->map_->getOccDist(mi, mj) / self->non_free_space_radius_;
        double f = self->non_free_space_factor_;
        f += delta_d * (1.0 - self->non_free_space_factor_);
        sample->weight *= f;
      }
      rv += sample->weight;
    }
  }
  return rv;
}

////////////////////////////////////////////////////////////////////////////////
// Determine the probability for the given pose
double
PlanarScanner::calcBeamModel(PlanarData *data, PFSampleSet* set)
{
  PlanarScanner *self;
  int i, j, step;
  double z, pz;
  double p;
  double map_range;
  double obs_range, obs_bearing;
  double total_weight;
  PFSample *sample;
  PFVector pose;

  self = (PlanarScanner*) data->sensor_;

  total_weight = 0.0;

  // Compute the sample weights
  for (j = 0; j < set->sample_count; j++)
  {
    sample = set->samples + j;
    pose = sample->pose;

    // Take account of the planar scanner pose relative to the robot
    pose = PFVector::pfVectorCoordAdd(self->planar_scanner_pose_, pose);

    p = 1.0;

    step = (data->range_count_ - 1) / (self->max_beams_ - 1);
    for (i = 0; i < data->range_count_; i += step)
    {
      obs_range = data->ranges_[i][0];
      obs_bearing = data->ranges_[i][1];

      // Compute the range according to the map
      map_range = self->map_->calcRange(pose.v[0], pose.v[1],
                                 pose.v[2] + obs_bearing, data->range_max_);
      pz = 0.0;

      // Part 1: good, but noisy, hit
      z = obs_range - map_range;
      pz += self->z_hit_ * exp(-(z * z) / (2 * self->sigma_hit_ * self->sigma_hit_));

      // Part 2: short reading from unexpected obstacle (e.g., a person)
      if(z < 0)
        pz += self->z_short_ * self->lambda_short_ * exp(-self->lambda_short_*obs_range);

      // Part 3: Failure to detect obstacle, reported as max-range
      if(obs_range == data->range_max_)
        pz += self->z_max_ * 1.0;

      // Part 4: Random measurements
      if(obs_range < data->range_max_)
        pz += self->z_rand_ * 1.0/data->range_max_;

      // TODO: outlier rejection for short readings

      assert(pz <= 1.0);
      assert(pz >= 0.0);
      //      p *= pz;
      // here we have an ad-hoc weighting scheme for combining beam probs
      // works well, though...
      p += pz*pz*pz;
    }

    sample->weight *= p;
    total_weight += sample->weight;
  }

  return(total_weight);
}

double
PlanarScanner::calcLikelihoodFieldModel(PlanarData *data, PFSampleSet* set)
{
  PlanarScanner *self;
  int i, j, step;
  double z, pz;
  double p;
  double obs_range, obs_bearing;
  double total_weight;
  PFSample *sample;
  PFVector pose;
  PFVector hit;

  self = (PlanarScanner*) data->sensor_;

  total_weight = 0.0;

  // Compute the sample weights
  for (j = 0; j < set->sample_count; j++)
  {
    sample = set->samples + j;
    pose = sample->pose;

    // Take account of the planar scanner pose relative to the robot
    pose = PFVector::pfVectorCoordAdd(self->planar_scanner_pose_, pose);

    p = 1.0;

    // Pre-compute a couple of things
    double z_hit_denom = 2 * self->sigma_hit_ * self->sigma_hit_;
    double z_rand_mult = 1.0/data->range_max_;

    step = (data->range_count_ - 1) / (self->max_beams_ - 1);

    // Step size must be at least 1
    if(step < 1)
      step = 1;

    for (i = 0; i < data->range_count_; i += step)
    {
      obs_range = data->ranges_[i][0];
      obs_bearing = data->ranges_[i][1];

      // This model ignores max range readings
      if(obs_range >= data->range_max_)
        continue;

      // Check for NaN
      if(obs_range != obs_range)
        continue;

      pz = 0.0;

      // Compute the endpoint of the beam
      hit.v[0] = pose.v[0] + obs_range * cos(pose.v[2] + obs_bearing);
      hit.v[1] = pose.v[1] + obs_range * sin(pose.v[2] + obs_bearing);

      // Convert to map_ grid coords.
      int mi, mj;
      self->map_->convertWorldToMap({hit.v[0], hit.v[1]}, &self->map_vec_);
      mi = self->map_vec_[0];
      mj = self->map_vec_[1];

      // Part 1: Get distance from the hit to closest obstacle.
      // Off-map penalized as max distance
      if(!self->map_->isValid({mi, mj}))
        z = self->map_->getMaxOccDist();
      else
        z = self->map_->getOccDist(mi,mj);
      // Gaussian model
      // NOTE: this should have a normalization of 1/(sqrt(2pi)*sigma)
      pz += self->z_hit_ * exp(-(z * z) / z_hit_denom);
      // Part 2: random measurements
      pz += self->z_rand_ * z_rand_mult;

      // TODO: outlier rejection for short readings

      assert(pz <= 1.0);
      assert(pz >= 0.0);
      //      p *= pz;
      // here we have an ad-hoc weighting scheme for combining beam probs
      // works well, though...
      // TODO: investigate schemes for combining beam probs
      p += pz*pz*pz;
    }

    sample->weight *= p;
    total_weight += sample->weight;
  }

  return(total_weight);
}

double
PlanarScanner::calcLikelihoodFieldModelProb(PlanarData *data, PFSampleSet* set)
{
  PlanarScanner *self;
  int i, j, step;
  double z, pz;
  double log_p;
  double obs_range, obs_bearing;
  double total_weight;
  PFSample *sample;
  PFVector pose;
  PFVector hit;

  self = (PlanarScanner*) data->sensor_;

  total_weight = 0.0;

  step = ceil((data->range_count_) / static_cast<double>(self->max_beams_));

  // Step size must be at least 1
  if(step < 1)
    step = 1;

  // Pre-compute a couple of things
  double z_hit_denom = 2 * self->sigma_hit_ * self->sigma_hit_;
  double z_rand_mult = 1.0/data->range_max_;

  double max_dist_prob = exp(-(self->map_->getMaxOccDist() * self->map_->getMaxOccDist()) / z_hit_denom);

  //Beam skipping - ignores beams for which a majoirty of particles do not agree with the map
  //prevents correct particles from getting down weighted because of unexpected obstacles
  //such as humans

  bool do_beamskip = self->do_beamskip_;
  double beam_skip_distance = self->beam_skip_distance_;
  double beam_skip_threshold = self->beam_skip_threshold_;

  //we only do beam skipping if the filter has converged
  if(do_beamskip && !set->converged){
    do_beamskip = false;
  }

  //we need a count the no of particles for which the beam agreed with the map
  int *obs_count = new int[self->max_beams_]();

  //we also need a mask of which observations to integrate (to decide which beams to integrate to all particles)
  bool *obs_mask = new bool[self->max_beams_]();

  int beam_ind = 0;

  //realloc indicates if we need to reallocate the temp data structure needed to do beamskipping
  bool realloc = false;

  if(do_beamskip){
    if(self->max_obs_ < self->max_beams_){
      realloc = true;
    }

    if(self->max_samples_ < set->sample_count){
      realloc = true;
    }

    if(realloc){
      self->reallocTempData(set->sample_count, self->max_beams_);
      fprintf(stderr, "Reallocing temp weights %d - %d\n", self->max_samples_, self->max_obs_);
    }
  }

  // Compute the sample weights
  for (j = 0; j < set->sample_count; j++)
  {
    sample = set->samples + j;
    pose = sample->pose;

    // Take account of the planar scanner pose relative to the robot
    pose = PFVector::pfVectorCoordAdd(self->planar_scanner_pose_, pose);

    log_p = 0;

    beam_ind = 0;

    for (i = 0; i < data->range_count_; i += step, beam_ind++)
    {
      obs_range = data->ranges_[i][0];
      obs_bearing = data->ranges_[i][1];

      // This model ignores max range readings
      if(obs_range >= data->range_max_){
        continue;
      }

      // Check for NaN
      if(obs_range != obs_range){
        continue;
      }

      pz = 0.0;

      // Compute the endpoint of the beam
      hit.v[0] = pose.v[0] + obs_range * cos(pose.v[2] + obs_bearing);
      hit.v[1] = pose.v[1] + obs_range * sin(pose.v[2] + obs_bearing);

      // Convert to map grid coords.
      int mi, mj;
      self->map_->convertWorldToMap({hit.v[0], hit.v[1]}, &self->map_vec_);
      mi = self->map_vec_[0];
      mj = self->map_vec_[1];

      // Part 1: Get distance from the hit to closest obstacle.
      // Off-map penalized as max distance

      if(!self->map_->isValid({mi, mj})){
	pz += self->z_hit_ * max_dist_prob;
      }
      else{
	z = self->map_->getOccDist(mi,mj);
	if(z < beam_skip_distance){
	  obs_count[beam_ind] += 1;
	}
	pz += self->z_hit_ * exp(-(z * z) / z_hit_denom);
      }

      // Gaussian model
      // NOTE: this should have a normalization of 1/(sqrt(2pi)*sigma)

      // Part 2: random measurements
      pz += self->z_rand_ * z_rand_mult;

      assert(pz <= 1.0); 
      assert(pz >= 0.0);

      // TODO: outlier rejection for short readings

      if(!do_beamskip){
	log_p += log(pz);
      }
      else{
	self->temp_obs_[j][beam_ind] = pz;
      }
    }
    if(!do_beamskip){
      sample->weight *= exp(log_p);
      total_weight += sample->weight;
    }
  }
  
  if(do_beamskip){
    int skipped_beam_count = 0; 
    for (beam_ind = 0; beam_ind < self->max_beams_; beam_ind++){
      if((obs_count[beam_ind] / static_cast<double>(set->sample_count)) > beam_skip_threshold){
	obs_mask[beam_ind] = true;
      }
      else{
	obs_mask[beam_ind] = false;
	skipped_beam_count++; 
      }
    }

    //we check if there is at least a critical number of beams that agreed with the map 
    //otherwise it probably indicates that the filter converged to a wrong solution
    //if that's the case we integrate all the beams and hope the filter might converge to 
    //the right solution
    bool error = false; 

    if(skipped_beam_count >= (beam_ind * self->beam_skip_error_threshold_)){
      fprintf(stderr, "Over %f%% of the observations were not in the map - pf may have converged to "
              "wrong pose - integrating all observations\n", (100 * self->beam_skip_error_threshold_));
      error = true; 
    }

    for (j = 0; j < set->sample_count; j++)
      {
	sample = set->samples + j;
	pose = sample->pose;

	log_p = 0;

	for (beam_ind = 0; beam_ind < self->max_beams_; beam_ind++){
	  if(error || obs_mask[beam_ind]){
	    log_p += log(self->temp_obs_[j][beam_ind]);
	  }
	}
	
	sample->weight *= exp(log_p);
	
	total_weight += sample->weight;
      }      
  }

  delete [] obs_count; 
  delete [] obs_mask;
  return(total_weight);
}

double
PlanarScanner::applyGompertz( double p )
{
  // shift and scale p
  p = p * this->input_scale_ + this->input_shift_;
  // apply gompertz
  p = this->gompertz_a_ * exp( -1.0 * this->gompertz_b_ * exp( -1.0 * this->gompertz_c_ * p ) );
  // shift output
  p += this->output_shift_;

  return p;
}

double
PlanarScanner::calcLikelihoodFieldModelGompertz(PlanarData *data, PFSampleSet* set)
{
  PlanarScanner *self;
  int i, j, step;
  double z, pz;
  double p;
  double obs_range, obs_bearing;
  double total_weight;
  PFSample *sample;
  PFVector pose;
  PFVector hit;

  self = (PlanarScanner*) data->sensor_;

  total_weight = 0.0;

  // Compute the sample weights
  for (j = 0; j < set->sample_count; j++)
  {
    sample = set->samples + j;
    pose = sample->pose;

    // Take account of the planar scanner pose relative to the robot
    pose = PFVector::pfVectorCoordAdd(self->planar_scanner_pose_, pose);

    // Pre-compute a couple of things
    double z_hit_denom = 2 * self->sigma_hit_ * self->sigma_hit_;

    step = (data->range_count_ - 1) / (self->max_beams_ - 1);

    // Step size must be at least 1
    if(step < 1)
      step = 1;

    int valid_beams = 0;
    double sum_pz = 0.0;
    for (i = 0; i < data->range_count_; i += step)
    {
      obs_range = data->ranges_[i][0];
      obs_bearing = data->ranges_[i][1];

      // This model ignores max range readings
      if(obs_range >= data->range_max_)
        continue;

      // Check for NaN
      if(obs_range != obs_range)
        continue;

      valid_beams++;
      pz = 0.0;

      // Compute the endpoint of the beam
      hit.v[0] = pose.v[0] + obs_range * cos(pose.v[2] + obs_bearing);
      hit.v[1] = pose.v[1] + obs_range * sin(pose.v[2] + obs_bearing);

      // Convert to map grid coords.
      int mi, mj;
      self->map_->convertWorldToMap({hit.v[0], hit.v[1]}, &self->map_vec_);
      mi = self->map_vec_[0];
      mj = self->map_vec_[1];
      // Part 1: Get distance from the hit to closest obstacle.
      // Off-map penalized as max distance
      if(!self->map_->isValid({mi, mj}))
        z = self->map_->getMaxOccDist();
      else
        z = self->map_->getOccDist(mi,mj);
      // Gaussian model
      pz += self->z_hit_ * exp(-(z * z) / z_hit_denom);
      // Part 2: random measurements
      pz += self->z_rand_;

      sum_pz += pz;
    }

    if (valid_beams > 0)
    {
      p = sum_pz / valid_beams;
      p = self->applyGompertz(p);
    }
    else
    {
      // Hmm. No valid beams. Don't change the weight.
      p = 1.0;
    }

    sample->weight *= p;
    total_weight += sample->weight;
  }

  return(total_weight);
}

void
PlanarScanner::reallocTempData(int new_max_samples, int new_max_obs){
  if(temp_obs_){
    for(int k=0; k < max_samples_; k++){
      delete [] temp_obs_[k];
    }
    delete []temp_obs_; 
  }
  max_obs_ = new_max_obs; 
  max_samples_ = fmax(max_samples_, new_max_samples); 

  temp_obs_ = new double*[max_samples_]();
  for(int k=0; k < max_samples_; k++){
    temp_obs_[k] = new double[max_obs_]();
  }
}

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
/**************************************************************************
 * Desc: Simple particle filter for localization.
 * Author: Andrew Howard
 * Maintainter: Tyler Buchman (tyler_buchman@jabil.com)
 *************************************************************************/

#include "pf/particle_filter.h"

#include <math.h>
#include <ros/assert.h>
#include <stddef.h>
#include <stdlib.h>
#include <time.h>

#include <cstdlib>

#include "pf/pdf_gaussian.h"

using namespace amcl;

// Create a new filter
ParticleFilter::ParticleFilter(int min_samples, int max_samples, double alpha_slow, double alpha_fast,
                               PFInitModelFnPtr random_pose_fn, void *random_pose_data)
{
  int i, j;
  PFSampleSet *set;
  PFSample *sample;

  srand48(time(NULL));

  this->resample_model_ = PF_RESAMPLE_MULTINOMIAL;
  this->random_pose_fn_ = random_pose_fn;
  this->random_pose_data_ = random_pose_data;

  this->min_samples_ = min_samples;
  this->max_samples_ = max_samples;

  // Control parameters for the population size calculation.  [err] is
  // the max error between the true distribution and the estimated
  // distribution.  [z] is the upper standard normal quantile for (1 -
  // p), where p is the probability that the error on the estimated
  // distrubition will be less than [err].
  pop_err_ = 0.01;
  pop_z_ = 3;
  dist_threshold_ = 0.5;

  current_set_ = 0;
  for (j = 0; j < 2; j++)
  {
    set = sets_ + j;

    set->sample_count = max_samples_;
    set->samples = (PFSample*)std::calloc(max_samples_, sizeof(PFSample));

    for (i = 0; i < set->sample_count; i++)
    {
      sample = set->samples + i;
      sample->pose.v[0] = 0.0;
      sample->pose.v[1] = 0.0;
      sample->pose.v[2] = 0.0;
      sample->weight = 1.0 / max_samples_;
    }

    // HACK: is 3 times max_samples_ enough?
    set->kdtree = new PFKDTree(3 * max_samples_);

    set->cluster_count = 0;
    set->cluster_max_count = max_samples_;
    set->clusters = (PFCluster*)std::calloc(set->cluster_max_count, sizeof(PFCluster));

    set->mean = PFVector();
    set->cov = PFMatrix();
  }

  this->w_slow_ = 0.0;
  this->w_fast_ = 0.0;

  this->alpha_slow_ = alpha_slow;
  this->alpha_fast_ = alpha_fast;

  //set converged to 0
  initConverged();
}

// Free an existing filter
ParticleFilter::~ParticleFilter()
{
  int i;

  for (i = 0; i < 2; i++)
  {
    free(sets_[i].clusters);
    sets_[i].kdtree->~PFKDTree();
    free(sets_[i].samples);
  }
}

void
ParticleFilter::setResampleModel(PFResampleModelType resample_model)
{
  this->resample_model_ = resample_model;
}

// Initialize the filter using a guassian
void
ParticleFilter::init(PFVector mean, PFMatrix cov)
{
  int i;
  PFSampleSet *set;
  PFSample *sample;

  set = sets_ + current_set_;

  // Create the kd tree for adaptive sampling
  set->kdtree->clearKDTree();

  set->sample_count = max_samples_;

  PDFGaussian pdf(mean, cov);

  // Compute the new sample poses
  for (i = 0; i < set->sample_count; i++)
  {
    sample = set->samples + i;
    sample->weight = 1.0 / max_samples_;
    sample->pose = pdf.sample();

    // Add sample to histogram
    set->kdtree->insertPose(sample->pose, sample->weight);
  }

  w_slow_ = w_fast_ = 0.0;

  // Re-compute cluster statistics
  clusterStats(set);

  //set converged to false
  initConverged();
}

// Initialize the filter using some model
void
ParticleFilter::initModel(PFInitModelFnPtr init_fn, void *init_data)
{
  int i;
  PFSampleSet *set;
  PFSample *sample;

  set = sets_ + current_set_;

  // Create the kd tree for adaptive sampling
  set->kdtree->clearKDTree();

  set->sample_count = max_samples_;

  // Compute the new sample poses
  for (i = 0; i < set->sample_count; i++)
  {
    sample = set->samples + i;
    sample->weight = 1.0 / max_samples_;
    sample->pose = (*init_fn) (init_data);

    // Add sample to histogram
    set->kdtree->insertPose(sample->pose, sample->weight);
  }

  w_slow_ = w_fast_ = 0.0;

  // Re-compute cluster statistics
  clusterStats(set);

  //set converged to false
  initConverged();
}

void
ParticleFilter::initConverged(){
  PFSampleSet *set;
  set = sets_ + current_set_;
  set->converged = 0;
  converged_ = false;
}

bool
ParticleFilter::updateConverged()
{
  int i;
  PFSampleSet *set;
  PFSample *sample;
  double total;

  set = sets_ + current_set_;
  double mean_x = 0, mean_y = 0;

  for (i = 0; i < set->sample_count; i++){
    sample = set->samples + i;

    mean_x += sample->pose.v[0];
    mean_y += sample->pose.v[1];
  }
  mean_x /= set->sample_count;
  mean_y /= set->sample_count;

  for (i = 0; i < set->sample_count; i++){
    sample = set->samples + i;
    if(fabs(sample->pose.v[0] - mean_x) > dist_threshold_ ||
       fabs(sample->pose.v[1] - mean_y) > dist_threshold_){
      set->converged = 0;
      converged_ = false;
      return 0;
    }
  }
  set->converged = 1;
  converged_ = 1;
  return 1;
}

// Update the filter with some new action
void
ParticleFilter::updateAction(PFActionModelFnPtr action_fn, void *action_data)
{
  PFSampleSet *set;

  set = sets_ + current_set_;

  (*action_fn) (action_data, set);

  return;
}

// Update the filter with some new sensor observation
void
ParticleFilter::updateSensor(PFSensorModelFnPtr sensor_fn, void *sensor_data)
{
  int i;
  PFSampleSet *set;
  PFSample *sample;
  double total;

  set = sets_ + current_set_;

  // Compute the sample weights
  total = (*sensor_fn) (sensor_data, set);

  if (total > 0.0)
  {
    // Normalize weights
    double w_avg=0.0;
    for (i = 0; i < set->sample_count; i++)
    {
      sample = set->samples + i;
      w_avg += sample->weight;
      sample->weight /= total;
    }
    // Update running averages of likelihood of samples (Prob Rob p258)
    w_avg /= set->sample_count;
    if(w_slow_ == 0.0)
      w_slow_ = w_avg;
    else
      w_slow_ += alpha_slow_ * (w_avg - w_slow_);
    if(w_fast_ == 0.0)
      w_fast_ = w_avg;
    else
      w_fast_ += alpha_fast_ * (w_avg - w_fast_);
  }
  else
  {
    // Handle zero total
    for (i = 0; i < set->sample_count; i++)
    {
      sample = set->samples + i;
      sample->weight = 1.0 / set->sample_count;
    }
  }
}

double
ParticleFilter::resampleSystematic(double w_diff)
{
  int i;
  double total;
  PFSampleSet *set_a, *set_b;
  PFSample *sample_a, *sample_b;

  double* c;

  set_a = sets_ + current_set_;
  set_b = sets_ + (current_set_ + 1) % 2;

  // Build up cumulative probability table for resampling.
  c = (double*)malloc(sizeof(double)*(set_a->sample_count+1));
  c[0] = 0.0;
  for(i=0;i<set_a->sample_count;i++)
  {
    c[i+1] = c[i]+set_a->samples[i].weight;
  }

  // Draw samples from set a to create set b.
  total = 0;
  set_b->sample_count = 0;

  // Approximate set_b's leaf_count from set_a's
  int new_count = resampleLimit(set_a->kdtree->getLeafCount());
  // Try to add particles for randomness.
  // No need to throw away our (possibly good) particles when we have free space in the filter for random ones.
  if(w_diff > 0.0)
  {
    new_count *= (1.0 + w_diff);
    if (new_count > max_samples_)
    {
      new_count = max_samples_;
    }
  }
  set_b->sample_count = new_count;
  int n_rand = w_diff * set_b->sample_count;
  int n_systematic_sampled = set_b->sample_count - n_rand;

  // Find the starting point for systematic sampling.
  double systematic_sample_start = drand48();
  double systematic_sample_delta = 1.0 / n_systematic_sampled;
  int c_i;
  for(c_i=0;c_i<set_a->sample_count;c_i++)
  {
    if((c[c_i] <= systematic_sample_start) && (systematic_sample_start < c[c_i+1]))
      break;
  }
  for(i=0; i<n_rand; ++i)
  {
    sample_b = set_b->samples + i;
    sample_b->pose = (random_pose_fn_)(random_pose_data_);
    sample_b->weight = 1.0;
    total += sample_b->weight;
    // Add sample to histogram
    set_b->kdtree->insertPose(sample_b->pose, sample_b->weight);
  }
  double target = systematic_sample_start;
  for(; i<set_b->sample_count; ++i)
  {
    while(!((c[c_i] <= target) && (target < c[c_i+1])))
    {
      c_i++;
      if(c_i >= set_a->sample_count)
      {
        c_i=0;
      }
    }
    target += systematic_sample_delta;
    if(target > 1.0)
    {
      target = 0.0;
    }
    sample_a = set_a->samples + c_i;
    sample_b = set_b->samples + i;
    sample_b->pose = sample_a->pose;

    sample_b->weight = 1.0;
    total += sample_b->weight;

    // Add sample to histogram
    set_b->kdtree->insertPose(sample_b->pose, sample_b->weight);
  }

  free(c);
  return total;
}

double
ParticleFilter::resampleMultinomial(double w_diff)
{
  int i;
  double total;
  PFSampleSet *set_a, *set_b;
  PFSample *sample_a, *sample_b;

  //double count_inv;
  double* c;

  set_a = sets_ + current_set_;
  set_b = sets_ + (current_set_ + 1) % 2;

  // Build up cumulative probability table for resampling.
  // TODO: Replace this with a more efficient procedure
  // (e.g., http://www.network-theory.co.uk/docs/gslref/GeneralDiscreteDistributions.html)
  c = (double*)malloc(sizeof(double)*(set_a->sample_count+1));
  c[0] = 0.0;
  for(i=0;i<set_a->sample_count;i++)
    c[i+1] = c[i]+set_a->samples[i].weight;

  // Draw samples from set a to create set b.
  total = 0;
  set_b->sample_count = 0;

  while(set_b->sample_count < max_samples_)
  {
    sample_b = set_b->samples + set_b->sample_count++;

    if(drand48() < w_diff)
      sample_b->pose = (random_pose_fn_)(random_pose_data_);
    else
    {
      // Naive discrete event sampler
      double r;
      r = drand48();
      for(i=0;i<set_a->sample_count;i++)
      {
        if((c[i] <= r) && (r < c[i+1]))
          break;
      }
      ROS_ASSERT(i<set_a->sample_count);

      sample_a = set_a->samples + i;

      ROS_ASSERT(sample_a->weight > 0);

      // Add sample to list
      sample_b->pose = sample_a->pose;
    }

    sample_b->weight = 1.0;
    total += sample_b->weight;

    // Add sample to histogram
    set_b->kdtree->insertPose(sample_b->pose, sample_b->weight);

    // See if we have enough samples yet
    if (set_b->sample_count > resampleLimit(set_b->kdtree->getLeafCount()))
      break;
  }

  free(c);
  return total;
}

// Resample the distribution
void
ParticleFilter::updateResample()
{
  int i;
  double total;
  PFSampleSet *set_a, *set_b;
  PFSample *sample_a, *sample_b;

  double w_diff;

  set_a = sets_ + current_set_;
  set_b = sets_ + (current_set_ + 1) % 2;

  // Create the kd tree for adaptive sampling
  set_b->kdtree->clearKDTree();

  w_diff = 1.0 - w_fast_ / w_slow_;
  if(w_diff < 0.0)
    w_diff = 0.0;

  switch(resample_model_)
  {
    case PF_RESAMPLE_MULTINOMIAL:
    default:
      total = resampleMultinomial(w_diff);
      break;
    case PF_RESAMPLE_SYSTEMATIC:
      total = resampleSystematic(w_diff);
      break;
  }

  // Reset averages, to avoid spiraling off into complete randomness.
  if(w_diff > 0.0)
    w_slow_ = w_fast_ = 0.0;

  // Normalize weights
  for (i = 0; i < set_b->sample_count; i++)
  {
    sample_b = set_b->samples + i;
    sample_b->weight /= total;
  }

  // Re-compute cluster statistics
  clusterStats(set_b);

  // Use the newly created sample set
  current_set_ = (current_set_ + 1) % 2;

  updateConverged();
}

// Compute the required number of samples, given that there are k bins
// with samples in them.  This is taken directly from Fox et al.
int
ParticleFilter::resampleLimit(int k)
{
  double a, b, c, x;
  int n;

  if (k <= 1)
    return max_samples_;

  a = 1;
  b = 2 / (9 * ((double) k - 1));
  c = sqrt(2 / (9 * ((double) k - 1))) * pop_z_;
  x = a - b + c;

  n = (int) ceil((k - 1) / (2 * pop_err_) * x * x * x);

  if (n < min_samples_)
    return min_samples_;
  if (n > max_samples_)
    return max_samples_;

  if (n < min_samples_)
    return min_samples_;
  if (n > max_samples_)
    return max_samples_;

  return n;
}

// Re-compute the cluster statistics for a sample set
void
ParticleFilter::clusterStats(PFSampleSet *set)
{
  int i, j, k, cidx;
  PFSample *sample;
  PFCluster *cluster;

  // Workspace
  double m[4], c[2][2];
  size_t count;
  double weight;

  // Cluster the samples
  set->kdtree->cluster();

  // Initialize cluster stats
  set->cluster_count = 0;

  for (i = 0; i < set->cluster_max_count; i++)
  {
    cluster = set->clusters + i;
    cluster->count = 0;
    cluster->weight = 0;
    cluster->mean = PFVector();
    cluster->cov = PFMatrix();

    for (j = 0; j < 4; j++)
      cluster->m[j] = 0.0;
    for (j = 0; j < 2; j++)
      for (k = 0; k < 2; k++)
        cluster->c[j][k] = 0.0;
  }

  // Initialize overall filter stats
  count = 0;
  weight = 0.0;
  set->mean = PFVector();
  set->cov = PFMatrix();
  for (j = 0; j < 4; j++)
    m[j] = 0.0;
  for (j = 0; j < 2; j++)
    for (k = 0; k < 2; k++)
      c[j][k] = 0.0;

  // Compute cluster stats
  for (i = 0; i < set->sample_count; i++)
  {
    sample = set->samples + i;

    //printf("%d %f %f %f\n", i, sample->pose.v[0], sample->pose.v[1], sample->pose.v[2]);

    // Get the cluster label for this sample
    cidx = set->kdtree->getCluster(sample->pose);
    ROS_ASSERT(cidx >= 0);
    if (cidx >= set->cluster_max_count)
      continue;
    if (cidx + 1 > set->cluster_count)
      set->cluster_count = cidx + 1;

    cluster = set->clusters + cidx;

    cluster->count += 1;
    cluster->weight += sample->weight;

    count += 1;
    weight += sample->weight;

    // Compute mean
    cluster->m[0] += sample->weight * sample->pose.v[0];
    cluster->m[1] += sample->weight * sample->pose.v[1];
    cluster->m[2] += sample->weight * cos(sample->pose.v[2]);
    cluster->m[3] += sample->weight * sin(sample->pose.v[2]);

    m[0] += sample->weight * sample->pose.v[0];
    m[1] += sample->weight * sample->pose.v[1];
    m[2] += sample->weight * cos(sample->pose.v[2]);
    m[3] += sample->weight * sin(sample->pose.v[2]);

    // Compute covariance in linear components
    for (j = 0; j < 2; j++)
      for (k = 0; k < 2; k++)
      {
        cluster->c[j][k] += sample->weight * sample->pose.v[j] * sample->pose.v[k];
        c[j][k] += sample->weight * sample->pose.v[j] * sample->pose.v[k];
      }
  }

  // Normalize
  for (i = 0; i < set->cluster_count; i++)
  {
    cluster = set->clusters + i;

    cluster->mean.v[0] = cluster->m[0] / cluster->weight;
    cluster->mean.v[1] = cluster->m[1] / cluster->weight;
    cluster->mean.v[2] = atan2(cluster->m[3], cluster->m[2]);

    cluster->cov = PFMatrix();

    // Covariance in linear components
    for (j = 0; j < 2; j++)
      for (k = 0; k < 2; k++)
        cluster->cov.m[j][k] = cluster->c[j][k] / cluster->weight -
          cluster->mean.v[j] * cluster->mean.v[k];

    // Covariance in angular components; I think this is the correct
    // formula for circular statistics.
    cluster->cov.m[2][2] = -2 * log(sqrt(cluster->m[2] * cluster->m[2] +
                                         cluster->m[3] * cluster->m[3]));
  }

  // Compute overall filter stats
  set->mean.v[0] = m[0] / weight;
  set->mean.v[1] = m[1] / weight;
  set->mean.v[2] = atan2(m[3], m[2]);

  // Covariance in linear components
  for (j = 0; j < 2; j++)
    for (k = 0; k < 2; k++)
      set->cov.m[j][k] = c[j][k] / weight - set->mean.v[j] * set->mean.v[k];

  // Covariance in angular components; I think this is the correct
  // formula for circular statistics.
  set->cov.m[2][2] = -2 * log(sqrt(m[2] * m[2] + m[3] * m[3]));
}

// Compute the CEP statistics (mean and variance).
void
ParticleFilter::getCepStats(PFVector *mean, double *var)
{
  int i;
  double mn, mx, my, mrr;
  PFSampleSet *set;
  PFSample *sample;

  set = sets_ + current_set_;

  mn = 0.0;
  mx = 0.0;
  my = 0.0;
  mrr = 0.0;

  for (i = 0; i < set->sample_count; i++)
  {
    sample = set->samples + i;

    mn += sample->weight;
    mx += sample->weight * sample->pose.v[0];
    my += sample->weight * sample->pose.v[1];
    mrr += sample->weight * sample->pose.v[0] * sample->pose.v[0];
    mrr += sample->weight * sample->pose.v[1] * sample->pose.v[1];
  }

  mean->v[0] = mx / mn;
  mean->v[1] = my / mn;
  mean->v[2] = 0.0;

  *var = mrr / mn - (mx * mx / (mn * mn) + my * my / (mn * mn));
}

// Get the statistics for a particular cluster.
bool
ParticleFilter::getClusterStats(int clabel, double *weight, PFVector *mean, PFMatrix *cov)
{
  PFSampleSet *set;
  PFCluster *cluster;

  set = sets_ + current_set_;

  if (clabel >= set->cluster_count)
    return false;
  cluster = set->clusters + clabel;

  *weight = cluster->weight;
  *mean = cluster->mean;
  *cov = cluster->cov;

  return true;
}

// sets population size parameters
void
ParticleFilter::setPopulationSizeParameters(double pop_err, double pop_z)
{
  this->pop_err_ = pop_err;
  this->pop_z_ = pop_z;
}

void
ParticleFilter::setDecayRates(double alpha_slow, double alpha_fast)
{
  this->alpha_slow_ = alpha_slow;
  this->alpha_fast_ = alpha_fast;
}

// gets pointer to current sample set
PFSampleSet*
ParticleFilter::getCurrentSet()
{
  return sets_ + current_set_;
}

// returns whether the particle filter has converged
bool
ParticleFilter::isConverged()
{
  return converged_;
}

// sets whether the particle filter has converged
void
ParticleFilter::setConverged(bool converged)
{
  this->converged_ = converged;
}

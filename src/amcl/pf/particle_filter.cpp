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

#include "pf/particle_filter.h"

#include <stdlib.h>

#include <cmath>
#include <cstddef>
#include <cstdlib>

#include <ros/assert.h>

#include "pf/pdf_gaussian.h"
#include "sensors/sensor.h"

namespace badger_amcl
{

// Create a new filter
ParticleFilter::ParticleFilter(int min_samples, int max_samples, double alpha_slow,
                               double alpha_fast, std::function<PFVector()> random_pose_fn)
{
  int i, j;
  std::shared_ptr<PFSampleSet> set;
  PFSample* sample;

  resample_model_ = PF_RESAMPLE_MULTINOMIAL;
  random_pose_fn_ = random_pose_fn;

  min_samples_ = min_samples;
  max_samples_ = max_samples;

  // Control parameters for the population size calculation.  [err] is
  // the max error between the true distribution and the estimated
  // distribution.  [z] is the upper standard normal quantile for (1 -
  // p), where p is the probability that the error on the estimated
  // distrubition will be less than [err].
  pop_err_ = 0.01;
  pop_z_ = 3;
  dist_threshold_ = 0.5;

  sets_ = { std::make_shared<PFSampleSet>(), std::make_shared<PFSampleSet>() };

  current_set_ = 0;
  for (j = 0; j < 2; j++)
  {
    set = sets_[j];

    set->sample_count = max_samples_;
    set->samples = std::vector<PFSample>(max_samples);

    for (i = 0; i < set->sample_count; i++)
    {
      sample = &(set->samples[i]);
      sample->pose.v[0] = 0.0;
      sample->pose.v[1] = 0.0;
      sample->pose.v[2] = 0.0;
      sample->weight = 1.0 / max_samples_;
    }

    set->kdtree = std::make_shared<PFKDTree>();

    set->cluster_count = 0;
    set->cluster_max_count = max_samples_;
    set->clusters = std::vector<PFCluster>(set->cluster_max_count);

    set->mean = PFVector();
    set->cov = PFMatrix();
  }

  w_slow_ = 0.0;
  w_fast_ = 0.0;

  alpha_slow_ = alpha_slow;
  alpha_fast_ = alpha_fast;

  // set converged to 0
  initConverged();
}

void ParticleFilter::setResampleModel(PFResampleModelType resample_model)
{
  resample_model_ = resample_model;
}

// Initialize the filter using a guassian
void ParticleFilter::init(PFVector mean, PFMatrix cov)
{
  int i;
  std::shared_ptr<PFSampleSet> set;
  PFSample* sample;
  set = sets_[current_set_];
  // Create the kd tree for adaptive sampling
  set->kdtree->clearKDTree();
  set->sample_count = max_samples_;
  PDFGaussian pdf(mean, cov);
  // Compute the new sample poses
  for (i = 0; i < set->sample_count; i++)
  {
    sample = &(set->samples[i]);
    sample->weight = 1.0 / max_samples_;
    sample->pose = pdf.sample();

    // Add sample to histogram
    set->kdtree->insertPose(sample->pose, sample->weight);
  }

  w_slow_ = w_fast_ = 0.0;

  // Re-compute cluster statistics
  clusterStats(set);

  // set converged to false
  initConverged();
}

// Initialize the filter using some model
void ParticleFilter::initModel(std::function<PFVector()> init_fn)
{
  int i;
  std::shared_ptr<PFSampleSet> set;
  PFSample* sample;

  set = sets_[current_set_];

  // Create the kd tree for adaptive sampling
  set->kdtree->clearKDTree();
  set->sample_count = max_samples_;

  // Compute the new sample poses
  for (i = 0; i < set->sample_count; i++)
  {
    sample = &(set->samples[i]);
    sample->weight = 1.0 / max_samples_;
    sample->pose = init_fn();
    // Add sample to histogram
    set->kdtree->insertPose(sample->pose, sample->weight);
  }
  w_slow_ = w_fast_ = 0.0;
  // Re-compute cluster statistics
  clusterStats(set);

  // set converged to false
  initConverged();
}

void ParticleFilter::initConverged()
{
  std::shared_ptr<PFSampleSet> set;
  set = sets_[current_set_];
  set->converged = 0;
  converged_ = false;
}

bool ParticleFilter::updateConverged()
{
  int i;
  std::shared_ptr<PFSampleSet> set;
  PFSample* sample;
  double total;

  set = sets_[current_set_];
  double mean_x = 0, mean_y = 0;

  for (i = 0; i < set->sample_count; i++)
  {
    sample = &(set->samples[i]);

    mean_x += sample->pose.v[0];
    mean_y += sample->pose.v[1];
  }
  mean_x /= set->sample_count;
  mean_y /= set->sample_count;

  for (i = 0; i < set->sample_count; i++)
  {
    sample = &(set->samples[i]);
    if (std::fabs(sample->pose.v[0] - mean_x) > dist_threshold_
        || std::fabs(sample->pose.v[1] - mean_y) > dist_threshold_)
    {
      set->converged = 0;
      converged_ = false;
      return 0;
    }
  }
  set->converged = 1;
  converged_ = 1;
  return 1;
}

// Update the filter with some new sensor observation
void ParticleFilter::updateSensor(std::function<double(std::shared_ptr<SensorData>, std::shared_ptr<PFSampleSet>)
                                               > sensor_fn,
                                  std::shared_ptr<SensorData> sensor_data)
{
  int i;
  std::shared_ptr<PFSampleSet> update_set;
  PFSample* update_sample;
  double total;

  update_set = sets_[current_set_];

  // Compute the sample weights
  total = sensor_fn(sensor_data, update_set);

  if (total > 0.0)
  {
    // Normalize weights
    double w_avg = 0.0;
    for (i = 0; i < update_set->sample_count; i++)
    {
      update_sample = &(update_set->samples[i]);
      w_avg += update_sample->weight;
      update_sample->weight /= total;
    }
    // Update running averages of likelihood of samples (from Probabilistic Robotics 'Augmented_MCL' algorithm)
    w_avg /= update_set->sample_count;
    if (w_slow_ == 0.0)
      w_slow_ = w_avg;
    else
      w_slow_ += alpha_slow_ * (w_avg - w_slow_);
    if (w_fast_ == 0.0)
      w_fast_ = w_avg;
    else
      w_fast_ += alpha_fast_ * (w_avg - w_fast_);
  }
  else
  {
    // Handle zero total
    for (i = 0; i < update_set->sample_count; i++)
    {
      update_sample = &(update_set->samples[i]);
      update_sample->weight = 1.0 / update_set->sample_count;
    }
  }
}

double ParticleFilter::resampleSystematic(double w_diff)
{
  int i;
  double total;
  std::shared_ptr<PFSampleSet> set_a, set_b;
  PFSample *sample_a, *sample_b;

  set_a = sets_[current_set_];
  set_b = sets_[(current_set_ + 1) % 2];

  // Build up cumulative probability table for resampling.
  std::vector<double> c = std::vector<double>(set_a->sample_count + 1);
  c[0] = 0.0;
  for (i = 0; i < set_a->sample_count; i++)
  {
    c[i + 1] = c[i] + set_a->samples[i].weight;
  }

  // Draw samples from set a to create set b.
  total = 0;
  set_b->sample_count = 0;

  // Approximate set_b's leaf_count from set_a's
  int new_count = resampleLimit(set_a->kdtree->getLeafCount());
  // Try to add particles for randomness.
  // No need to throw away our (possibly good) particles when we have free space in the filter
  // for random ones.
  if (w_diff > 0.0)
  {
    new_count *= (1.0 + w_diff);
    if (new_count > max_samples_)
    {
      new_count = max_samples_;
    }
  }
  set_b->sample_count = new_count;
  int num_random_poses = w_diff * set_b->sample_count;
  int num_systematic_sampled_poses = set_b->sample_count - num_random_poses;

  // Find the starting point for systematic sampling.
  double systematic_sample_start = drand48();
  double systematic_sample_delta = 1.0 / num_systematic_sampled_poses;
  int c_i;
  for (c_i = 0; c_i < set_a->sample_count; c_i++)
  {
    if ((c[c_i] <= systematic_sample_start) && (systematic_sample_start < c[c_i + 1]))
      break;
  }
  for (i = 0; i < num_random_poses; ++i)
  {
    sample_b = &(set_b->samples[i]);
    sample_b->pose = random_pose_fn_();
    sample_b->weight = 1.0;
    total += sample_b->weight;
    // Add sample to histogram
    set_b->kdtree->insertPose(sample_b->pose, sample_b->weight);
  }
  double target = systematic_sample_start;
  for (; i < set_b->sample_count; ++i)
  {
    while (!((c[c_i] <= target) && (target < c[c_i + 1])))
    {
      c_i++;
      if (c_i >= set_a->sample_count)
      {
        c_i = 0;
      }
    }
    target += systematic_sample_delta;
    if (target > 1.0)
    {
      target -= 1.0;
    }
    sample_a = &(set_a->samples[c_i]);
    sample_b = &(set_b->samples[i]);
    sample_b->pose = sample_a->pose;

    sample_b->weight = 1.0;
    total += sample_b->weight;

    // Add sample to histogram
    set_b->kdtree->insertPose(sample_b->pose, sample_b->weight);
  }

  return total;
}

double ParticleFilter::resampleMultinomial(double w_diff)
{
  int i;
  double total;
  std::shared_ptr<PFSampleSet> set_a, set_b;
  PFSample *sample_a, *sample_b;

  // double count_inv;
  std::vector<double> c;

  set_a = sets_[current_set_];
  set_b = sets_[(current_set_ + 1) % 2];

  // Build up cumulative probability table for resampling.
  // TODO: Replace this with a more efficient procedure
  // (e.g., http://www.network-theory.co.uk/docs/gslref/GeneralDiscreteDistributions.html)
  c = std::vector<double>(set_a->sample_count + 1);
  c[0] = 0.0;
  for (i = 0; i < set_a->sample_count; i++)
    c[i + 1] = c[i] + set_a->samples[i].weight;

  // Draw samples from set a to create set b.
  total = 0;
  set_b->sample_count = 0;

  while (set_b->sample_count < max_samples_)
  {
    sample_b = &(set_b->samples[set_b->sample_count++]);

    if (drand48() < w_diff)
    {
      sample_b->pose = random_pose_fn_();
    }
    else
    {
      // Naive discrete event sampler
      double r;
      r = drand48();
      for (i = 0; i < set_a->sample_count; i++)
      {
        if ((c[i] <= r) && (r < c[i + 1]))
          break;
      }
      ROS_ASSERT(i < set_a->sample_count);

      sample_a = &(set_a->samples[i]);

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
  return total;
}

// Resample the distribution
void ParticleFilter::updateResample()
{
  int i;
  double total;
  std::shared_ptr<PFSampleSet> set_a, set_b;
  PFSample *sample_a, *sample_b;

  double w_diff;

  set_a = sets_[current_set_];
  set_b = sets_[(current_set_ + 1) % 2];

  // Create the kd tree for adaptive sampling
  set_b->kdtree->clearKDTree();

  w_diff = 1.0 - w_fast_ / w_slow_;
  if (w_diff < 0.0)
    w_diff = 0.0;

  switch (resample_model_)
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
  if (w_diff > 0.0)
    w_slow_ = w_fast_ = 0.0;

  // Normalize weights
  for (i = 0; i < set_b->sample_count; i++)
  {
    sample_b = &(set_b->samples[i]);
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
int ParticleFilter::resampleLimit(int k)
{
  double a, b, c, x, kd;
  int n;

  if (k <= 1)
    return max_samples_;

  kd = static_cast<double>(k);
  a = 1;
  b = 2 / (9 * (kd - 1));
  c = std::sqrt(2 / (9 * (kd - 1))) * pop_z_;
  x = a - b + c;

  n = static_cast<int>(std::ceil((k - 1) / (2 * pop_err_) * x * x * x));

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
void ParticleFilter::clusterStats(std::shared_ptr<PFSampleSet> set)
{
  int i, j, k, cidx;
  PFSample* sample;
  PFCluster* cluster;

  // Workspace
  double m[4], c[2][2];
  std::size_t count;
  double weight;

  // Cluster the samples
  set->kdtree->cluster();

  // Initialize cluster stats
  set->cluster_count = 0;

  for (i = 0; i < set->cluster_max_count; i++)
  {
    cluster = &(set->clusters[i]);
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
    sample = &(set->samples[i]);

    // Get the cluster label for this sample
    cidx = set->kdtree->getCluster(sample->pose);
    ROS_ASSERT(cidx >= 0);
    if (cidx >= set->cluster_max_count)
      continue;
    if (cidx + 1 > set->cluster_count)
      set->cluster_count = cidx + 1;

    cluster = &(set->clusters[cidx]);

    cluster->count += 1;
    cluster->weight += sample->weight;

    count += 1;
    weight += sample->weight;

    // Compute mean
    cluster->m[0] += sample->weight * sample->pose.v[0];
    cluster->m[1] += sample->weight * sample->pose.v[1];
    cluster->m[2] += sample->weight * std::cos(sample->pose.v[2]);
    cluster->m[3] += sample->weight * std::sin(sample->pose.v[2]);

    m[0] += sample->weight * sample->pose.v[0];
    m[1] += sample->weight * sample->pose.v[1];
    m[2] += sample->weight * std::cos(sample->pose.v[2]);
    m[3] += sample->weight * std::sin(sample->pose.v[2]);

    // Compute covariance in linear components
    for (j = 0; j < 2; j++)
    {
      for (k = 0; k < 2; k++)
      {
        cluster->c[j][k] += sample->weight * sample->pose.v[j] * sample->pose.v[k];
        c[j][k] += sample->weight * sample->pose.v[j] * sample->pose.v[k];
      }
    }
  }

  // Normalize
  for (i = 0; i < set->cluster_count; i++)
  {
    cluster = &(set->clusters[i]);

    cluster->mean.v[0] = cluster->m[0] / cluster->weight;
    cluster->mean.v[1] = cluster->m[1] / cluster->weight;
    cluster->mean.v[2] = std::atan2(cluster->m[3], cluster->m[2]);

    cluster->cov = PFMatrix();

    // Covariance in linear components
    for (j = 0; j < 2; j++)
      for (k = 0; k < 2; k++)
        cluster->cov.m[j][k] = (cluster->c[j][k] / cluster->weight
                                - cluster->mean.v[j] * cluster->mean.v[k]);

    // Covariance in angular components; I think this is the correct
    // formula for circular statistics.
    cluster->cov.m[2][2] = -2 * std::log(std::sqrt(cluster->m[2] * cluster->m[2]
                                                   + cluster->m[3] * cluster->m[3]));
  }

  // Compute overall filter stats
  set->mean.v[0] = m[0] / weight;
  set->mean.v[1] = m[1] / weight;
  set->mean.v[2] = std::atan2(m[3], m[2]);

  // Covariance in linear components
  for (j = 0; j < 2; j++)
    for (k = 0; k < 2; k++)
      set->cov.m[j][k] = c[j][k] / weight - set->mean.v[j] * set->mean.v[k];

  // Covariance in angular components; I think this is the correct
  // formula for circular statistics.
  set->cov.m[2][2] = -2 * std::log(std::sqrt(m[2] * m[2] + m[3] * m[3]));
}

// Get the statistics for a particular cluster.
bool ParticleFilter::getClusterStats(int clabel, double* weight, PFVector* mean)
{
  std::shared_ptr<PFSampleSet> set;
  PFCluster* cluster;

  set = sets_[current_set_];

  if (clabel >= set->cluster_count)
    return false;
  cluster = &(set->clusters[clabel]);

  *weight = cluster->weight;
  *mean = cluster->mean;

  return true;
}

// sets population size parameters
void ParticleFilter::setPopulationSizeParameters(double pop_err, double pop_z)
{
  pop_err_ = pop_err;
  pop_z_ = pop_z;
}

void ParticleFilter::setDecayRates(double alpha_slow, double alpha_fast)
{
  alpha_slow_ = alpha_slow;
  alpha_fast_ = alpha_fast;
}

// gets pointer to current sample set
std::shared_ptr<PFSampleSet> ParticleFilter::getCurrentSet()
{
  return sets_[current_set_];
}

// returns whether the particle filter has converged
bool ParticleFilter::isConverged()
{
  return converged_;
}

// sets whether the particle filter has converged
void ParticleFilter::setConverged(bool converged)
{
  converged_ = converged;
}

}  // namespace amcl

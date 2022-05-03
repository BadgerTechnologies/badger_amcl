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

#ifndef AMCL_PF_PARTICLE_FILTER_H
#define AMCL_PF_PARTICLE_FILTER_H

#include <memory>
#include <vector>

#include <Eigen/Dense>

#include <pf/pf_kdtree.h>

namespace badger_amcl
{

enum PFResampleModelType
{
  PF_RESAMPLE_MULTINOMIAL,
  PF_RESAMPLE_SYSTEMATIC,
};

// Information for a single sample
struct PFSample
{
  // Pose represented by this sample
  Eigen::Vector3d pose;

  // Weight for this pose
  double weight;

};

// Information for a cluster of samples
struct PFCluster
{
  // Number of samples
  int count;

  // Total weight of samples in this cluster
  double weight;

  // Cluster statistics
  Eigen::Vector3d mean;
  Eigen::Matrix3d cov;

  // Workspace
  double m[4], c[2][2];

};

// Information for a set of samples
struct PFSampleSet
{
  // The samples
  int sample_count;
  std::vector<PFSample> samples;

  // A kdtree encoding the histogram
  std::shared_ptr<PFKDTree> kdtree;

  // Clusters
  int cluster_count, cluster_max_count;
  std::vector<PFCluster> clusters;

  // Filter statistics
  Eigen::Vector3d mean;
  Eigen::Matrix3d cov;
  int converged;
};

class SensorData;

// Information for an entire filter
class ParticleFilter
{
public:
  // Create a new filter
  ParticleFilter(int min_samples, int max_samples, double alpha_slow, double alpha_fast, double convergence_threshold,
                 std::function<Eigen::Vector3d()> random_pose_fn);

  // Set the resample model
  void setResampleModel(PFResampleModelType resample_model);

  // Initialize the filter using a guassian to generate initial poses
  void initWithGaussian(const Eigen::Vector3d& mean, const Eigen::Matrix3d& cov);

  // Initialize the filter using a function to generate initial poses
  void initWithPoseFn(std::function<Eigen::Vector3d()> pose_fn);

  // Update the filter with some new sensor observation
  void updateSensor(std::function<double(std::shared_ptr<SensorData>,
                                         std::shared_ptr<PFSampleSet>)> sensor_fn_ptr,
                    std::shared_ptr<SensorData> sensor_data);

  // Resample the distribution
  void updateResample();

  // Compute the statistics for a particular cluster.  Returns false if
  // there is no such cluster.
  bool getClusterStats(int cluster, double* weight, Eigen::Vector3d* mean);

  // sets population size parameters
  void setPopulationSizeParameters(double pop_err, double pop_z);

  // sets decay rates for running averages
  void setDecayRates(double alpha_slow, double alpha_fast);

  // gets pointer to current sample set
  std::shared_ptr<PFSampleSet> getCurrentSet();

  // getter for whether the particle filter has converged
  bool isConverged();

private:
  // Compute the required number of samples, given that there are k bins
  // with samples in them.
  int resampleLimit(int k);

  double resampleSystematic(double w_diff);
  double resampleMultinomial(double w_diff);

  // sets the current set and pf converged values to false
  void initConverged();

  // calculate if the particle filter has converged -
  // and sets the converged flag in the current set and the pf
  void updateConverged();

  // Re-compute the cluster statistics for a sample set
  void computeClusterStatsForSet(std::shared_ptr<PFSampleSet> sample_set);
  void initCluster(PFCluster* cluster);
  void normalizeCluster(PFCluster* cluster);
  int getClusterIndexOfSampleInSet(std::shared_ptr<PFSampleSet> set, PFSample* sample);
  void addSampleStatsToCluster(const PFSample*, PFCluster* cluster);
  void addSampleStatsToSet(const PFSample* sample, double* weight, double* m, double* c);
  void computeSetStats(double weight, const double m[], const double c[],
                       std::shared_ptr<PFSampleSet> set);

  PFResampleModelType resample_model_;

  // This min and max number of samples
  int min_samples_, max_samples_;

  // Running averages, slow and fast, of likelihood
  double w_slow_, w_fast_;

  // Function used to draw random pose samples
  std::function<Eigen::Vector3d()> random_pose_fn_;

  double dist_threshold_;  // distance threshold in each axis over which the pf is considered to not be converged

  double convergence_threshold; // Global localization convergence threshold

  // Population size parameters
  double pop_err_, pop_z_;

  // Decay rates for running averages
  double alpha_slow_, alpha_fast_;

  // The sample sets. We keep two sets and use [current_set]
  // to identify the active set.
  int current_set_;
  std::vector<std::shared_ptr<PFSampleSet>> sets_;

  bool converged_;
};

}  // namespace amcl

#endif  // AMCL_PF_PARTICLE_FILTER_H

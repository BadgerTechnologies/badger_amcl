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

#ifndef AMCL_PARTICLE_FILTER_H
#define AMCL_PARTICLE_FILTER_H

#include "pf/pf_kdtree.h"
#include "pf/pf_vector.h"

namespace amcl
{

typedef enum
{
  PF_RESAMPLE_MULTINOMIAL,
  PF_RESAMPLE_SYSTEMATIC,
} PFResampleModelType;

// Information for a single sample
typedef struct
{
  // Pose represented by this sample
  PFVector pose;

  // Weight for this pose
  double weight;
  
} PFSample;

// Information for a cluster of samples
typedef struct
{
  // Number of samples
  int count;

  // Total weight of samples in this cluster
  double weight;

  // Cluster statistics
  PFVector mean;
  PFMatrix cov;

  // Workspace
  double m[4], c[2][2];
  
} PFCluster;

// Information for a set of samples
typedef struct
{
  // The samples
  int sample_count;
  PFSample *samples;

  // A kdtree encoding the histogram
  PFKDTree *kdtree;

  // Clusters
  int cluster_count, cluster_max_count;
  PFCluster *clusters;

  // Filter statistics
  PFVector mean;
  PFMatrix cov;
  int converged; 
} PFSampleSet;

// Function prototype for the initialization model; generates a sample pose from
// an appropriate distribution.
typedef PFVector (*PFInitModelFnPtr) (void *init_data);

// Function prototype for the action model; generates a sample pose from
// an appropriate distribution
typedef void (*PFActionModelFnPtr) (void *action_data, PFSampleSet* set);

// Function prototype for the sensor model; determines the probability
// for the given set of sample poses.
typedef double (*PFSensorModelFnPtr) (void *sensor_data, PFSampleSet* set);

// Information for an entire filter
class ParticleFilter
{
  public:
    // Create a new filter
    ParticleFilter(int min_samples, int max_samples, double alpha_slow, double alpha_fast,
         PFInitModelFnPtr random_pose_fn, void *random_pose_data);

    // Free an existing filter
    ~ParticleFilter();

    // Set the resample model
    void setResampleModel(PFResampleModelType resample_model);

    double resampleSystematic(double w_diff);

    double resampleMultinomial(double w_diff);

    // Initialize the filter using a guassian
    void init(PFVector mean, PFMatrix cov);

    // Initialize the filter using some model
    void initModel(PFInitModelFnPtr init_fn, void *init_data);

    // Update the filter with some new action
    void updateAction(PFActionModelFnPtr action_fn, void *action_data);

    // Update the filter with some new sensor observation
    void updateSensor(PFSensorModelFnPtr sensor_fn, void *sensor_data);

    // Resample the distribution
    void updateResample();

    // Compute the CEP statistics (mean and variance).
    void getCepStats(PFVector *mean, double *var);

    // Compute the statistics for a particular cluster.  Returns false if
    // there is no such cluster.
    bool getClusterStats(int cluster, double *weight, PFVector *mean, PFMatrix *cov);

    //calculate if the particle filter has converged - 
    //and sets the converged flag in the current set and the pf 
    bool updateConverged();

    //sets the current set and pf converged values to zero
    void initConverged();

    // sets population size parameters
    void setPopulationSizeParameters(double pop_err, double pop_z);

    // sets decay rates for running averages
    void setDecayRates(double alpha_slow, double alpha_fast);

    // gets pointer to current sample set
    PFSampleSet* getCurrentSet();

    // getter and setter for whether the particle filter has converged
    bool isConverged();
    void setConverged(bool converged);

  private:
    // Compute the required number of samples, given that there are k bins
    // with samples in them.
    int resampleLimit(int k);

    // Re-compute the cluster statistics for a sample set
    void clusterStats(PFSampleSet *sample_set);

    PFResampleModelType resample_model_;

    // This min and max number of samples
    int min_samples_, max_samples_;

    // Running averages, slow and fast, of likelihood
    double w_slow_, w_fast_;

    // Function used to draw random pose samples
    PFInitModelFnPtr random_pose_fn_;
    void *random_pose_data_;

    double dist_threshold_; //distance threshold in each axis over which the pf is considered to not be converged

    // Population size parameters
    double pop_err_, pop_z_;

    // Decay rates for running averages
    double alpha_slow_, alpha_fast_;

    // The sample sets.  We keep two sets and use [current_set]
    // to identify the active set.
    int current_set_;
    PFSampleSet sets_[2];

    bool converged_;
};

}

#endif

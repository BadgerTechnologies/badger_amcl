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
 * Date: 10 Dec 2002
 * CVS: $Id: pf.h 3293 2005-11-19 08:37:45Z gerkey $
 *************************************************************************/

#ifndef PF_H
#define PF_H

#include "pf_vector.h"
#include "pf_kdtree.h"

namespace amcl
{

typedef enum
{
  PF_RESAMPLE_MULTINOMIAL,
  PF_RESAMPLE_SYSTEMATIC,
} pf_resample_model_t;

// Forward declarations
struct _pf_sample_set_t;

// Function prototype for the initialization model; generates a sample pose from
// an appropriate distribution.
typedef PFVector (*pf_init_model_fn_t) (void *init_data);

// Function prototype for the action model; generates a sample pose from
// an appropriate distribution
typedef void (*pf_action_model_fn_t) (void *action_data, 
                                      struct _pf_sample_set_t* set);

// Function prototype for the sensor model; determines the probability
// for the given set of sample poses.
typedef double (*pf_sensor_model_fn_t) (void *sensor_data, 
                                        struct _pf_sample_set_t* set);


// Information for a single sample
typedef struct
{
  // Pose represented by this sample
  PFVector pose;

  // Weight for this pose
  double weight;
  
} pf_sample_t;


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
  
} pf_cluster_t;


// Information for a set of samples
typedef struct _pf_sample_set_t
{
  // The samples
  int sample_count;
  pf_sample_t *samples;

  // A kdtree encoding the histogram
  KDTree *kdtree;

  // Clusters
  int cluster_count, cluster_max_count;
  pf_cluster_t *clusters;

  // Filter statistics
  PFVector mean;
  PFMatrix cov;
  int converged; 
} pf_sample_set_t;


// Information for an entire filter
class ParticleFilter
{
  public:
    // Create a new filter
    ParticleFilter(int min_samples, int max_samples, double alpha_slow, double alpha_fast,
         pf_init_model_fn_t random_pose_fn, void *random_pose_data);

    // Free an existing filter
    ~ParticleFilter();

    // Set the resample model
    void set_resample_model(pf_resample_model_t resample_model);

    double resample_systematic(double w_diff);

    double resample_multinomial(double w_diff);

    // Initialize the filter using a guassian
    void init(PFVector mean, PFMatrix cov);

    // Initialize the filter using some model
    void init_model(pf_init_model_fn_t init_fn, void *init_data);

    // Update the filter with some new action
    void update_action(pf_action_model_fn_t action_fn, void *action_data);

    // Update the filter with some new sensor observation
    void update_sensor(pf_sensor_model_fn_t sensor_fn, void *sensor_data);

    // Resample the distribution
    void update_resample();

    // Compute the CEP statistics (mean and variance).
    void get_cep_stats(PFVector *mean, double *var);

    // Compute the statistics for a particular cluster.  Returns false if
    // there is no such cluster.
    bool get_cluster_stats(int cluster, double *weight, PFVector *mean, PFMatrix *cov);

    //calculate if the particle filter has converged - 
    //and sets the converged flag in the current set and the pf 
    bool update_converged();

    //sets the current set and pf converged values to zero
    void init_converged();

    pf_resample_model_t resample_model;

    // This min and max number of samples
    int min_samples, max_samples;

    // Population size parameters
    double pop_err, pop_z;
  
    // The sample sets.  We keep two sets and use [current_set]
    // to identify the active set.
    int current_set;
    pf_sample_set_t sets[2];

    // Running averages, slow and fast, of likelihood
    double w_slow, w_fast;

    // Decay rates for running averages
    double alpha_slow, alpha_fast;

    // Function used to draw random pose samples
    pf_init_model_fn_t random_pose_fn;
    void *random_pose_data;

    double dist_threshold; //distance threshold in each axis over which the pf is considered to not be converged
    int converged; 

  private:
    // Compute the required number of samples, given that there are k bins
    // with samples in them.
    int resample_limit(int k);

    // Re-compute the cluster statistics for a sample set
    void cluster_stats(pf_sample_set_t *sample_set);
};

}

#endif

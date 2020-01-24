/*
 *  Copyright (C) 2000  Brian Gerkey et al.
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
// Desc: Odometry sensor model for AMCL
// Author: Andrew Howard
// Maintainer: Tyler Buchman (tyler_buchman@jabil.com)
//
///////////////////////////////////////////////////////////////////////////

#ifndef AMCL_SENSORS_ODOM_H
#define AMCL_SENSORS_ODOM_H

#include "sensors/sensor.h"

#include <memory>

#include "pf/particle_filter.h"
#include "pf/pf_vector.h"

namespace amcl
{
typedef enum {
  ODOM_MODEL_DIFF,
  ODOM_MODEL_OMNI,
  ODOM_MODEL_DIFF_CORRECTED,
  ODOM_MODEL_OMNI_CORRECTED,
  ODOM_MODEL_GAUSSIAN,
} OdomModelType;

// Odometric sensor data
class OdomData : public SensorData
{
public:
  // Odometric pose
  PFVector pose;
  // Change in odometric pose
  PFVector delta;
  // Total absolute motion (relative to base)
  PFVector absolute_motion;
};

// Odometric sensor model
class Odom : public Sensor
{
  // Default constructor
public:
  Odom();

  void setModelDiff(double alpha1, double alpha2, double alpha3, double alpha4);

  void setModelOmni(double alpha1, double alpha2, double alpha3, double alpha4, double alpha5);

  void setModelGaussian(double alpha1, double alpha2, double alpha3, double alpha4, double alpha5);

  void setModel(OdomModelType type, double alpha1, double alpha2, double alpha3, double alpha4, double alpha5 = 0);

  // Update the filter based on the action model.  Returns true if the filter
  // has been updated.
  virtual bool updateAction(std::shared_ptr<ParticleFilter> pf, std::shared_ptr<SensorData> data);

private:
  double normalize(double z);
  double angleDiff(double a, double b);
  // Model type
  OdomModelType model_type_;

  // Drift parameters
  double alpha1_, alpha2_, alpha3_, alpha4_, alpha5_;
};
}  // namespace amcl

#endif  // AMCL_SENSORS_ODOM_H

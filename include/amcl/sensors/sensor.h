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
 */
///////////////////////////////////////////////////////////////////////////
//
// Desc: Adaptive Monte-Carlo localization
// Author: Andrew Howard
//
///////////////////////////////////////////////////////////////////////////

#ifndef AMCL_SENSORS_SENSOR_H
#define AMCL_SENSORS_SENSOR_H

#include <memory>

#include "pf/particle_filter.h"

namespace amcl
{
// Forward declarations
class SensorData;

// Base class for all AMCL sensors
class Sensor
{
public:
  virtual ~Sensor() = default;

  virtual bool updateAction(std::shared_ptr<ParticleFilter> pf,
                            std::shared_ptr<SensorData> data) { return false; };

  // Update the filter based on the sensor model.  Returns true if the
  // filter has been updated.
  virtual bool updateSensor(std::shared_ptr<ParticleFilter> pf,
                            std::shared_ptr<SensorData> data) { return false; };
};

// Base class for all AMCL sensor measurements
class SensorData
{
  // Pointer to sensor that generated the data
public:
  virtual ~SensorData() = default;
};

}  // namespace amcl

#endif  // AMCL_SENSORS_SENSOR_H

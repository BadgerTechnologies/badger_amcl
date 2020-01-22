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
// Desc: AMCL sensor
// Author: Andrew Howard
// Maintainer: Tyler Buchman (tyler_buchman@jabil.com)
//
///////////////////////////////////////////////////////////////////////////

#include "sensors/sensor.h"

using namespace amcl;

////////////////////////////////////////////////////////////////////////////////
// Default constructor
Sensor::Sensor()
{
  return;
}

Sensor::~Sensor()
{
}

////////////////////////////////////////////////////////////////////////////////
// Apply the action model
bool Sensor::updateAction(std::shared_ptr<ParticleFilter> pf, std::shared_ptr<SensorData> data)
{
  return false;
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the filter
bool Sensor::initSensor(std::shared_ptr<ParticleFilter> pf, std::shared_ptr<SensorData> data)
{
  return false;
}

////////////////////////////////////////////////////////////////////////////////
// Apply the sensor model
bool Sensor::updateSensor(std::shared_ptr<ParticleFilter> pf, std::shared_ptr<SensorData> data)
{
  return false;
}

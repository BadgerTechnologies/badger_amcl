/*
 *  Copyright (C) 2020 Badger Technologies, LLC
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

#ifndef AMCL_NODE_NODE_ND_H
#define AMCL_NODE_NODE_ND_H

#include <Eigen/Dense>

#include "badger_amcl/AMCLConfig.h"

namespace badger_amcl
{

class NodeND
{
public:
  virtual ~NodeND() = default;
  virtual void reconfigure(AMCLConfig& config) = 0;
  virtual void globalLocalizationCallback() = 0;
  virtual double scorePose(const Eigen::Vector3d& p) = 0;
};

}  // namespace amcl

#endif  // AMCL_NODE_NODE_ND_H

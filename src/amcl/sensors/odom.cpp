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

#include "sensors/odom.h"

#include <algorithm>
#include <cmath>

#include <angles/angles.h>

#include "pf/pdf_gaussian.h"

namespace badger_amcl
{

void Odom::initModel(double alpha1, double alpha2, double alpha3, double alpha4,
                    double alpha5)
{
  alpha1_ = alpha1;
  alpha2_ = alpha2;
  alpha3_ = alpha3;
  alpha4_ = alpha4;
  alpha5_ = alpha5;
}

bool Odom::updateAction(std::shared_ptr<ParticleFilter> pf, std::shared_ptr<SensorData> data)
{
  std::shared_ptr<OdomData> ndata;
  ndata = std::dynamic_pointer_cast<OdomData>(data);

  // Compute the new sample poses
  std::shared_ptr<PFSampleSet> set = pf->getCurrentSet();
  Eigen::Vector3d old_pose;
  old_pose[0] = ndata->pose[0] - ndata->delta[0];
  old_pose[1] = ndata->pose[1] - ndata->delta[1];
  old_pose[2] = ndata->pose[2] - ndata->delta[2];

  double delta_trans, delta_rot;
  double abs_delta_trans, abs_delta_strafe, abs_delta_rot;
  double abs_delta_trans2, abs_delta_strafe2, abs_delta_rot2;
  double delta_trans_hat, delta_rot_hat, delta_strafe_hat;

  delta_trans = std::sqrt(ndata->delta[0] * ndata->delta[0]
                          + ndata->delta[1] * ndata->delta[1]);
  delta_rot = ndata->delta[2];

  abs_delta_trans = ndata->absolute_motion[0];
  abs_delta_strafe = ndata->absolute_motion[1];
  abs_delta_rot = ndata->absolute_motion[2];

  abs_delta_trans2 = abs_delta_trans * abs_delta_trans;
  abs_delta_strafe2 = abs_delta_strafe * abs_delta_strafe;
  abs_delta_rot2 = abs_delta_rot * abs_delta_rot;

  double rot_hat_stddev = std::sqrt(alpha1_ * abs_delta_rot2 + alpha2_ * abs_delta_trans2);
  double trans_hat_stddev = std::sqrt(alpha3_ * abs_delta_trans2 + alpha4_ * abs_delta_rot2);
  double strafe_hat_stddev = std::sqrt(alpha4_ * abs_delta_rot2 + alpha5_ * abs_delta_strafe2);

  for (int i = 0; i < set->sample_count; i++)
  {
    PFSample* sample = &(set->samples[i]);

    // estimated direction pointed during motion
    double heading = sample->pose[2] + ndata->delta[2] / 2;
    double cs_heading = std::cos(heading);
    double sn_heading = std::sin(heading);

    // relative direction we moved
    double ndata_angle = std::atan2(ndata->delta[1], ndata->delta[0]);
    double delta_bearing = angleDiff(ndata_angle, old_pose[2]) + sample->pose[2];
    double cs_bearing = std::cos(delta_bearing);
    double sn_bearing = std::sin(delta_bearing);

    // Sample pose differences
    delta_trans_hat = PDFGaussian::draw(trans_hat_stddev);
    delta_strafe_hat = PDFGaussian::draw(strafe_hat_stddev);
    delta_rot_hat = PDFGaussian::draw(rot_hat_stddev);
    // Apply sampled update to particle pose
    sample->pose[0] += (delta_trans * cs_bearing);
    sample->pose[1] += (delta_trans * sn_bearing);
    sample->pose[2] += delta_rot;
    sample->pose[0] += (delta_trans_hat * cs_heading + delta_strafe_hat * sn_heading);
    sample->pose[1] += (delta_trans_hat * sn_heading - delta_strafe_hat * cs_heading);
    sample->pose[2] += delta_rot_hat;
  }
  return true;
}

double Odom::angleDiff(double a, double b)
{
  return angles::shortest_angular_distance(b, a);
}

}  // namespace badger_amcl

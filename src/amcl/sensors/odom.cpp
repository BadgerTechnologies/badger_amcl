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

void Odom::setModelDiff(double alpha1, double alpha2, double alpha3, double alpha4)
{
  model_type_ = ODOM_MODEL_DIFF;
  alpha1_ = alpha1;
  alpha2_ = alpha2;
  alpha3_ = alpha3;
  alpha4_ = alpha4;
}

void Odom::setModelOmni(double alpha1, double alpha2, double alpha3, double alpha4, double alpha5)
{
  model_type_ = ODOM_MODEL_OMNI;
  alpha1_ = alpha1;
  alpha2_ = alpha2;
  alpha3_ = alpha3;
  alpha4_ = alpha4;
  alpha5_ = alpha5;
}

void Odom::setModelGaussian(double alpha1, double alpha2, double alpha3, double alpha4,
                            double alpha5)
{
  model_type_ = ODOM_MODEL_GAUSSIAN;
  alpha1_ = alpha1;
  alpha2_ = alpha2;
  alpha3_ = alpha3;
  alpha4_ = alpha4;
  alpha5_ = alpha5;
}

void Odom::setModel(OdomModelType type, double alpha1, double alpha2, double alpha3, double alpha4,
                    double alpha5)
{
  model_type_ = type;
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

  switch (model_type_)
  {
    case ODOM_MODEL_OMNI:
    {
      double delta_trans, delta_rot, delta_bearing;
      double delta_trans_hat, delta_rot_hat, delta_strafe_hat;

      delta_trans = std::sqrt(ndata->delta[0] * ndata->delta[0]
                         + ndata->delta[1] * ndata->delta[1]);
      delta_rot = ndata->delta[2];

      // Precompute a couple of things
      double trans_hat_stddev = (alpha3_ * (delta_trans * delta_trans)
                                 + alpha1_ * (delta_rot * delta_rot));
      double rot_hat_stddev = (alpha4_ * (delta_rot * delta_rot)
                               + alpha2_ * (delta_trans * delta_trans));
      double strafe_hat_stddev = (alpha1_ * (delta_rot * delta_rot)
                                  + alpha5_ * (delta_trans * delta_trans));

      for (int i = 0; i < set->sample_count; i++)
      {
        PFSample* sample = &(set->samples[i]);

        double turn_angle = std::atan2(ndata->delta[1], ndata->delta[0]);
        delta_bearing = angleDiff(turn_angle, old_pose[2]) + sample->pose[2];
        double cs_bearing = std::cos(delta_bearing);
        double sn_bearing = std::sin(delta_bearing);

        // Sample pose differences
        delta_trans_hat = delta_trans + PDFGaussian::draw(trans_hat_stddev);
        delta_rot_hat = delta_rot + PDFGaussian::draw(rot_hat_stddev);
        delta_strafe_hat = 0 + PDFGaussian::draw(strafe_hat_stddev);
        // Apply sampled update to particle pose
        sample->pose[0] += (delta_trans_hat * cs_bearing + delta_strafe_hat * sn_bearing);
        sample->pose[1] += (delta_trans_hat * sn_bearing - delta_strafe_hat * cs_bearing);
        sample->pose[2] += delta_rot_hat;
      }
    }
    break;
    case ODOM_MODEL_DIFF:
    {
      // Implement sample_motion_odometry (Prob Rob p 136)
      double delta_rot1, delta_trans, delta_rot2;
      double delta_rot1_hat, delta_trans_hat, delta_rot2_hat;
      double delta_rot1_noise, delta_rot2_noise;

      // Avoid computing a bearing from two poses that are extremely near each
      // other (happens on in-place rotation).
      if (std::sqrt(ndata->delta[1] * ndata->delta[1]
                    + ndata->delta[0] * ndata->delta[0]) < 0.01)
        delta_rot1 = 0.0;
      else
        delta_rot1 = angleDiff(std::atan2(ndata->delta[1], ndata->delta[0]), old_pose[2]);
      delta_trans = std::sqrt(ndata->delta[0] * ndata->delta[0]
                              + ndata->delta[1] * ndata->delta[1]);
      delta_rot2 = angleDiff(ndata->delta[2], delta_rot1);

      // We want to treat backward and forward motion symmetrically for the
      // noise model to be applied below.  The standard model seems to assume
      // forward motion.
      delta_rot1_noise = std::min(std::fabs(angleDiff(delta_rot1, 0.0)),
                                  std::fabs(angleDiff(delta_rot1, M_PI)));
      delta_rot2_noise = std::min(std::fabs(angleDiff(delta_rot2, 0.0)),
                                  std::fabs(angleDiff(delta_rot2, M_PI)));

      for (int i = 0; i < set->sample_count; i++)
      {
        PFSample* sample = &(set->samples[i]);

        // Sample pose differences
        delta_rot1_hat = angleDiff(delta_rot1, PDFGaussian::draw(alpha1_ * delta_rot1_noise * delta_rot1_noise
                                                                 + alpha2_ * delta_trans * delta_trans));
        delta_trans_hat = (delta_trans - PDFGaussian::draw(alpha3_ * delta_trans * delta_trans +
                                                           alpha4_ * delta_rot1_noise * delta_rot1_noise +
                                                           alpha4_ * delta_rot2_noise * delta_rot2_noise));
        delta_rot2_hat = angleDiff(delta_rot2, PDFGaussian::draw(alpha1_ * delta_rot2_noise * delta_rot2_noise
                                                                 + alpha2_ * delta_trans * delta_trans));

        // Apply sampled update to particle pose
        sample->pose[0] += delta_trans_hat * std::cos(sample->pose[2] + delta_rot1_hat);
        sample->pose[1] += delta_trans_hat * std::sin(sample->pose[2] + delta_rot1_hat);
        sample->pose[2] += delta_rot1_hat + delta_rot2_hat;
      }
    }
    break;
    case ODOM_MODEL_OMNI_CORRECTED:
    {
      double delta_trans, delta_rot, delta_bearing;
      double delta_trans_hat, delta_rot_hat, delta_strafe_hat;

      delta_trans = std::sqrt(ndata->delta[0] * ndata->delta[0]
                              + ndata->delta[1] * ndata->delta[1]);
      delta_rot = ndata->delta[2];

      // Precompute a couple of things
      double trans_hat_stddev = std::sqrt(alpha3_ * (delta_trans * delta_trans)
                                          + alpha1_ * (delta_rot * delta_rot));
      double rot_hat_stddev = std::sqrt(alpha4_ * (delta_rot * delta_rot)
                                        + alpha2_ * (delta_trans * delta_trans));
      double strafe_hat_stddev = std::sqrt(alpha1_ * (delta_rot * delta_rot)
                                           + alpha5_ * (delta_trans * delta_trans));

      for (int i = 0; i < set->sample_count; i++)
      {
        PFSample* sample = &(set->samples[i]);

        double turn_angle = std::atan2(ndata->delta[1], ndata->delta[0]);
        delta_bearing = angleDiff(turn_angle,  old_pose[2]) + sample->pose[2];
        double cs_bearing = std::cos(delta_bearing);
        double sn_bearing = std::sin(delta_bearing);

        // Sample pose differences
        delta_trans_hat = delta_trans + PDFGaussian::draw(trans_hat_stddev);
        delta_rot_hat = delta_rot + PDFGaussian::draw(rot_hat_stddev);
        delta_strafe_hat = 0 + PDFGaussian::draw(strafe_hat_stddev);
        // Apply sampled update to particle pose
        sample->pose[0] += (delta_trans_hat * cs_bearing + delta_strafe_hat * sn_bearing);
        sample->pose[1] += (delta_trans_hat * sn_bearing - delta_strafe_hat * cs_bearing);
        sample->pose[2] += delta_rot_hat;
      }
    }
    break;
    case ODOM_MODEL_DIFF_CORRECTED:
    {
      // Implement sample_motion_odometry (Prob Rob p 136)
      double delta_rot1, delta_trans, delta_rot2;
      double delta_rot1_hat, delta_trans_hat, delta_rot2_hat;
      double delta_rot1_noise, delta_rot2_noise;

      // Avoid computing a bearing from two poses that are extremely near each
      // other (happens on in-place rotation).
      delta_trans = std::sqrt(ndata->delta[0] * ndata->delta[0]
                              + ndata->delta[1] * ndata->delta[1]);
      if (delta_trans < 0.01)
        delta_rot1 = 0.0;
      else
        delta_rot1 = angleDiff(std::atan2(ndata->delta[1], ndata->delta[0]), old_pose[2]);
      delta_rot2 = angleDiff(ndata->delta[2], delta_rot1);

      // We want to treat backward and forward motion symmetrically for the
      // noise model to be applied below.  The standard model seems to assume
      // forward motion.
      delta_rot1_noise = std::min(std::fabs(angleDiff(delta_rot1, 0.0)),
                                  std::fabs(angleDiff(delta_rot1, M_PI)));
      delta_rot2_noise = std::min(std::fabs(angleDiff(delta_rot2, 0.0)),
                                  std::fabs(angleDiff(delta_rot2, M_PI)));

      for (int i = 0; i < set->sample_count; i++)
      {
        PFSample* sample = &(set->samples[i]);

        // Sample pose differences
        double draw;
        draw = PDFGaussian::draw(std::sqrt(alpha1_ * delta_rot1_noise * delta_rot1_noise
                                           + alpha2_ * delta_trans * delta_trans));
        delta_rot1_hat = angleDiff(delta_rot1, draw);
        draw = PDFGaussian::draw(std::sqrt(alpha3_ * delta_trans * delta_trans
                                           + alpha4_ * delta_rot1_noise * delta_rot1_noise
                                           + alpha4_ * delta_rot2_noise * delta_rot2_noise));
        delta_trans_hat = (delta_trans - draw);
        draw = PDFGaussian::draw(std::sqrt(alpha1_ * delta_rot2_noise * delta_rot2_noise
                                           + alpha2_ * delta_trans * delta_trans));
        delta_rot2_hat = angleDiff(delta_rot2, draw);

        // Apply sampled update to particle pose
        sample->pose[0] += delta_trans_hat * std::cos(sample->pose[2] + delta_rot1_hat);
        sample->pose[1] += delta_trans_hat * std::sin(sample->pose[2] + delta_rot1_hat);
        sample->pose[2] += delta_rot1_hat + delta_rot2_hat;
      }
    }
    break;
    case ODOM_MODEL_GAUSSIAN:
    {
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
    }
    break;
  }
  return true;
}

double Odom::normalize(double z)
{
  return angles::normalize_angle(z);
}

double Odom::angleDiff(double a, double b)
{
  return angles::shortest_angular_distance(b, a);
}

}  // namespace amcl

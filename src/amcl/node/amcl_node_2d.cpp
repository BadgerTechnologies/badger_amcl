/*
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
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

#include <algorithm>
#include <vector>
#include <map>
#include <cmath>

#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

// Signal handling
#include <signal.h>

#include "map.h"
#include "occupancy_map.h"
#include "pf.h"
#include "amcl_odom.h"
#include "amcl_laser.h"
#include "amcl_node.h"

#include "ros/assert.h"

// roscpp
#include "ros/ros.h"

// Messages that I need
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/SetMap.h"
#include "std_srvs/Empty.h"

// For transform support
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "tf/tf.h"
#include "message_filters/subscriber.h"

// Dynamic_reconfigure
#include "dynamic_reconfigure/server.h"
#include "amcl/AMCLConfig.h"

#include "yaml-cpp/yaml.h"
#include <stdio.h>
#include <exception>

#include <badger_file_lib/atomic_ofstream.h>

using namespace amcl;

void
AmclNode::init2D()
{
  laser_ = NULL;
  last_laser_data_ = NULL;
  private_nh_.param("laser_min_range", laser_min_range_, -1.0);
  private_nh_.param("laser_max_range", laser_max_range_, -1.0);
  private_nh_.param("laser_max_beams", max_beams_, 30);
  private_nh_.param("laser_z_hit", z_hit_, 0.95);
  private_nh_.param("laser_z_short", z_short_, 0.1);
  private_nh_.param("laser_z_max", z_max_, 0.05);
  private_nh_.param("laser_z_rand", z_rand_, 0.05);
  private_nh_.param("laser_sigma_hit", sigma_hit_, 0.2);
  private_nh_.param("laser_lambda_short", lambda_short_, 0.1);
  private_nh_.param("laser_likelihood_max_dist", laser_likelihood_max_dist_, 2.0);
  private_nh_.param("laser_gompertz_a", laser_gompertz_a_, 1.0);
  private_nh_.param("laser_gompertz_b", laser_gompertz_b_, 1.0);
  private_nh_.param("laser_gompertz_c", laser_gompertz_c_, 1.0);
  private_nh_.param("laser_gompertz_input_shift", laser_gompertz_input_shift_, 0.0);
  private_nh_.param("laser_gompertz_input_scale", laser_gompertz_input_scale_, 1.0);
  private_nh_.param("laser_gompertz_output_shift", laser_gompertz_output_shift_, 0.0);
  private_nh_.param("laser_off_map_factor", laser_off_map_factor_, 1.0);
  private_nh_.param("laser_non_free_space_factor", laser_non_free_space_factor_, 1.0);
  private_nh_.param("laser_non_free_space_radius", laser_non_free_space_radius_, 0.0);
  private_nh_.param("global_localization_laser_off_map_factor", global_localization_laser_off_map_factor_, 1.0);
  private_nh_.param("global_localization_laser_non_free_space_factor", global_localization_laser_non_free_space_factor_, 1.0);
  std::string tmp_model_type;
  private_nh_.param("laser_model_type", tmp_model_type, std::string("likelihood_field"));
  if(tmp_model_type == "beam")
    laser_model_type_ = LASER_MODEL_BEAM;
  else if(tmp_model_type == "likelihood_field")
    laser_model_type_ = LASER_MODEL_LIKELIHOOD_FIELD;
  else if(tmp_model_type == "likelihood_field_prob")
    laser_model_type_ = LASER_MODEL_LIKELIHOOD_FIELD_PROB;
  else if(tmp_model_type == "likelihood_field_gompertz")
    laser_model_type_ = LASER_MODEL_LIKELIHOOD_FIELD_GOMPERTZ;
  else
  {
    ROS_WARN("Unknown laser model type \"%s\"; defaulting to likelihood_field model",
             tmp_model_type.c_str());
    laser_model_type_ = LASER_MODEL_LIKELIHOOD_FIELD;
  }

  laser_scan_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, scan_topic_, 100);
  laser_scan_filter_ =
          new tf::MessageFilter<sensor_msgs::LaserScan>(*laser_scan_sub_,
                                                        *tf_,
                                                        odom_frame_id_,
                                                        100);
  laser_scan_filter_->registerCallback(boost::bind(&AmclNode::laserReceived,
                                                   this, _1));

  // 15s timer to warn on lack of receipt of laser scans, #5209
  laser_check_interval_ = ros::Duration(15.0);
  check_laser_timer_ = nh_.createTimer(laser_check_interval_,
                                       boost::bind(&AmclNode::checkLaserReceived, this, _1));
}

void
AmclNode::checkLaserReceived(const ros::TimerEvent& event)
{
  ros::Duration d = ros::Time::now() - last_laser_received_ts_;
  if(d > laser_check_interval_)
  {
    ROS_WARN("No laser scan received (and thus no pose updates have been published) for %f seconds.  Verify that data is being published on the %s topic.",
             d.toSec(),
             ros::names::resolve(scan_topic_).c_str());
  }
}

/**
 * Convert an OccupancyGrid map message into the internal
 * representation.  This allocates an OccupancyMap and returns it.
 */
OccupancyMap*
AmclNode::convertMap( const nav_msgs::OccupancyGrid& map_msg )
{
  OccupancyMap* map = new OccupancyMap();
  ROS_ASSERT(map);
  std::vector<int> size_vec;
  double scale = map_msg.info.resolution / map_scale_up_factor_;
  size_vec.push_back(map_msg.info.width * map_scale_up_factor_);
  size_vec.push_back(map_msg.info.height * map_scale_up_factor_);
  map->setSize(size_vec);
  map->setScale(scale);
  std::vector<double> origin;
  origin.push_back(map_msg.info.origin.position.x + (size_vec[0] / 2) * scale);
  origin.push_back(map_msg.info.origin.position.y + (size_vec[1] / 2) * scale);
  map->setOrigin(origin);
  map->initCells(size_vec[0]*size_vec[1]);
  ROS_ASSERT(map->getCells());
  for(int y=0;y<size_vec[1];y++)
  {
    int i=y*size_vec[0];
    const int msg_row=(y/map_scale_up_factor_)*(map_msg.info.width);
    for(int x=0;x<size_vec[0];x++,i++)
    {
      const int msg_i=msg_row+x/map_scale_up_factor_;

      if(map_msg.data[msg_i] == 0)
        map->setCellOccState(i, -1);
      else if(map_msg.data[msg_i] == 100)
        map->setCellOccState(i, +1);
      else
        map->setCellOccState(i, 0);
    }
  }

  return map;
}

// Helper function to score a pose for uniform pose generation
double
AmclNode::scorePose2D(const pf_vector_t &p)
{
  if(this->last_laser_data_ == NULL)
  {
    // There is no data to match, so return a perfect match
    return 1.0;
  }
  // Create a fake "sample set" of just this pose to score it.
  pf_sample_t fake_sample;
  fake_sample.pose.v[0] = p.v[0];
  fake_sample.pose.v[1] = p.v[1];
  fake_sample.pose.v[2] = p.v[2];
  fake_sample.weight = 1.0;
  pf_sample_set_t fake_sample_set;
  fake_sample_set.sample_count = 1;
  fake_sample_set.samples = &fake_sample;
  fake_sample_set.converged = 0;
  AMCLLaser::ApplyModelToSampleSet(this->last_laser_data_, &fake_sample_set);
  return fake_sample.weight;
}

void
AmclNode::reconfigure2D(AMCLConfig &config)
{
  laser_min_range_ = config.laser_min_range;
  laser_max_range_ = config.laser_max_range;
  z_hit_ = config.laser_z_hit;
  z_short_ = config.laser_z_short;
  z_max_ = config.laser_z_max;
  z_rand_ = config.laser_z_rand;
  sigma_hit_ = config.laser_sigma_hit;
  lambda_short_ = config.laser_lambda_short;
  laser_likelihood_max_dist_ = config.laser_likelihood_max_dist;
  laser_off_map_factor_ = config.laser_off_map_factor;
  laser_non_free_space_factor_ = config.laser_non_free_space_factor;
  laser_non_free_space_radius_ = config.laser_non_free_space_radius;
  global_localization_laser_off_map_factor_ = config.global_localization_laser_off_map_factor;
  global_localization_laser_non_free_space_factor_ = config.global_localization_laser_non_free_space_factor;

  laser_gompertz_a_ = config.laser_gompertz_a;
  laser_gompertz_b_ = config.laser_gompertz_b;
  laser_gompertz_c_ = config.laser_gompertz_c;
  laser_gompertz_input_shift_ = config.laser_gompertz_input_shift;
  laser_gompertz_input_scale_ = config.laser_gompertz_input_scale;
  laser_gompertz_output_shift_ = config.laser_gompertz_output_shift;
  max_beams_ = config.laser_max_beams;
  if(config.laser_model_type == "beam")
    laser_model_type_ = LASER_MODEL_BEAM;
  else if(config.laser_model_type == "likelihood_field")
    laser_model_type_ = LASER_MODEL_LIKELIHOOD_FIELD;
  else if(config.laser_model_type == "likelihood_field_prob")
    laser_model_type_ = LASER_MODEL_LIKELIHOOD_FIELD_PROB;
  else if(config.laser_model_type == "likelihood_field_gompertz")
    laser_model_type_ = LASER_MODEL_LIKELIHOOD_FIELD_GOMPERTZ;
  if(config.odom_model_type == "diff")
    odom_model_type_ = ODOM_MODEL_DIFF;
  else if(config.odom_model_type == "omni")
    odom_model_type_ = ODOM_MODEL_OMNI;
  else if(config.odom_model_type == "diff-corrected")
    odom_model_type_ = ODOM_MODEL_DIFF_CORRECTED;
  else if(config.odom_model_type == "omni-corrected")
    odom_model_type_ = ODOM_MODEL_OMNI_CORRECTED;
  else if(config.odom_model_type == "gaussian");
    odom_model_type_ = ODOM_MODEL_GAUSSIAN;
  delete laser_;
  laser_ = new AMCLLaser(max_beams_, (OccupancyMap*)map_);
  ROS_ASSERT(laser_);
  if(laser_model_type_ == LASER_MODEL_BEAM)
    laser_->SetModelBeam(z_hit_, z_short_, z_max_, z_rand_,
                         sigma_hit_, lambda_short_, 0.0);
  else if(laser_model_type_ == LASER_MODEL_LIKELIHOOD_FIELD_PROB){
    ROS_INFO("Initializing likelihood field model; this can take some time on large maps...");
    laser_->SetModelLikelihoodFieldProb(z_hit_, z_rand_, sigma_hit_,
					laser_likelihood_max_dist_,
					do_beamskip_, beam_skip_distance_,
					beam_skip_threshold_, beam_skip_error_threshold_);
    ROS_INFO("Done initializing likelihood field model with probabilities.");
  }
  else if(laser_model_type_ == LASER_MODEL_LIKELIHOOD_FIELD){
    ROS_INFO("Initializing likelihood field model; this can take some time on large maps...");
    laser_->SetModelLikelihoodField(z_hit_, z_rand_, sigma_hit_,
                                    laser_likelihood_max_dist_);
    ROS_INFO("Done initializing likelihood field model.");
  }
  else if(laser_model_type_ == LASER_MODEL_LIKELIHOOD_FIELD_GOMPERTZ){
    ROS_INFO("Initializing likelihood field (gompertz) model; this can take some time on large maps...");
    laser_->SetModelLikelihoodFieldGompertz(z_hit_, z_rand_, sigma_hit_,
                                            laser_likelihood_max_dist_,
                                            laser_gompertz_a_,
                                            laser_gompertz_b_,
                                            laser_gompertz_c_,
                                            laser_gompertz_input_shift_,
                                            laser_gompertz_input_scale_,
                                            laser_gompertz_output_shift_);
    ROS_INFO("Gompertz key points by total laser scan match: "
        "0.0: %f, 0.25: %f, 0.5: %f, 0.75: %f, 1.0: %f",
        laser_->applyGompertz(z_rand_),
        laser_->applyGompertz(z_rand_ + z_hit_ * .25),
        laser_->applyGompertz(z_rand_ + z_hit_ * .5),
        laser_->applyGompertz(z_rand_ + z_hit_ * .75),
        laser_->applyGompertz(z_rand_ + z_hit_));
    ROS_INFO("Done initializing likelihood (gompertz) field model.");
  }
  laser_->SetMapFactors(laser_off_map_factor_, laser_non_free_space_factor_, laser_non_free_space_radius_);
  delete laser_scan_filter_;
  laser_scan_filter_ =
          new tf::MessageFilter<sensor_msgs::LaserScan>(*laser_scan_sub_,
                                                        *tf_,
                                                        odom_frame_id_,
                                                        100);
  laser_scan_filter_->registerCallback(boost::bind(&AmclNode::laserReceived,
                                                   this, _1));
}

void
AmclNode::mapReceived(const nav_msgs::OccupancyGridConstPtr& msg)
{
  if( first_map_only_ && first_map_received_ ) {
    return;
  }

  boost::recursive_mutex::scoped_lock cfl(configuration_mutex_);

  ROS_INFO("Received a %d X %d map @ %.3f m/pix\n",
           msg->info.width,
           msg->info.height,
           msg->info.resolution);

  freeMapDependentMemory();
  // Clear queued laser objects because they hold pointers to the existing map
  lasers_.clear();
  lasers_update_.clear();
  frame_to_laser_.clear();
  delete last_laser_data_;
  last_laser_data_ = NULL;

  map_ = convertMap(*msg);
  initFromNewMap();

  first_map_received_ = true;
}

void
AmclNode::initFromNewMap2D()
{
  // Laser
  delete laser_;
  laser_ = new AMCLLaser(max_beams_, (OccupancyMap*)map_);
  ROS_ASSERT(laser_);
  if(laser_model_type_ == LASER_MODEL_BEAM)
    laser_->SetModelBeam(z_hit_, z_short_, z_max_, z_rand_,
                         sigma_hit_, lambda_short_, 0.0);
  else if(laser_model_type_ == LASER_MODEL_LIKELIHOOD_FIELD_PROB){
    ROS_INFO("Initializing likelihood field model; this can take some time on large maps...");
    laser_->SetModelLikelihoodFieldProb(z_hit_, z_rand_, sigma_hit_,
					laser_likelihood_max_dist_,
					do_beamskip_, beam_skip_distance_,
					beam_skip_threshold_, beam_skip_error_threshold_);
    ROS_INFO("Done initializing likelihood field model.");
  }
  else if(laser_model_type_ == LASER_MODEL_LIKELIHOOD_FIELD_GOMPERTZ){
    ROS_INFO("Initializing likelihood field (gompertz) model; this can take some time on large maps...");
    laser_->SetModelLikelihoodFieldGompertz(z_hit_, z_rand_, sigma_hit_,
                                            laser_likelihood_max_dist_,
                                            laser_gompertz_a_,
                                            laser_gompertz_b_,
                                            laser_gompertz_c_,
                                            laser_gompertz_input_shift_,
                                            laser_gompertz_input_scale_,
                                            laser_gompertz_output_shift_);
    ROS_INFO("Gompertz key points by total laser scan match: "
        "0.0: %f, 0.25: %f, 0.5: %f, 0.75: %f, 1.0: %f",
        laser_->applyGompertz(z_rand_),
        laser_->applyGompertz(z_rand_ + z_hit_ * .25),
        laser_->applyGompertz(z_rand_ + z_hit_ * .5),
        laser_->applyGompertz(z_rand_ + z_hit_ * .75),
        laser_->applyGompertz(z_rand_ + z_hit_));
    ROS_INFO("Done initializing likelihood (gompertz) field model.");
  }
  else
  {
    ROS_INFO("Initializing likelihood field model; this can take some time on large maps...");
    laser_->SetModelLikelihoodField(z_hit_, z_rand_, sigma_hit_,
                                    laser_likelihood_max_dist_);
    ROS_INFO("Done initializing likelihood field model.");
  }
  laser_->SetMapFactors(laser_off_map_factor_, laser_non_free_space_factor_, laser_non_free_space_radius_);

  // Index of free space
  // Must be calculated after the occ_dist is setup by the laser model
  free_space_indices.resize(0);
  std::vector<int> size_vec = map_->getSize();
  for(int i = 0; i < size_vec[0]; i++)
    for(int j = 0; j < size_vec[1]; j++)
      if(((OccupancyMap*)map_)->getCells()[((OccupancyMap*)map_)->computeCellIndex(i,j)].occ_state == -1)
        if(((OccupancyMap*)map_)->occDist(i,j) > laser_non_free_space_radius_)
          free_space_indices.push_back(std::make_pair(i,j));
}

void
AmclNode::laserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan)
{
  last_laser_received_ts_ = ros::Time::now();
  if( map_ == NULL ) {
    return;
  }
  boost::recursive_mutex::scoped_lock cfl(configuration_mutex_);
  int laser_index = -1;

  // Handle corner cases like getting dynamically reconfigured or getting a
  // new map by de-activating the global localization parameters here if we are
  // no longer globally localizing.
  if(!global_localization_active_)
  {
    pf_->alpha_slow = alpha_slow_;
    pf_->alpha_fast = alpha_fast_;
    laser_->SetMapFactors(laser_off_map_factor_, laser_non_free_space_factor_, laser_non_free_space_radius_);
    for (auto& l : lasers_)
    {
      l->SetMapFactors(laser_off_map_factor_, laser_non_free_space_factor_, laser_non_free_space_radius_);
    }
  }

  // Do we have the base->base_laser Tx yet?
  if(frame_to_laser_.find(laser_scan->header.frame_id) == frame_to_laser_.end())
  {
    ROS_DEBUG("Setting up laser %d (frame_id=%s)\n", (int)frame_to_laser_.size(), laser_scan->header.frame_id.c_str());
    lasers_.push_back(new AMCLLaser(*laser_));
    lasers_update_.push_back(true);
    laser_index = frame_to_laser_.size();

    tf::Stamped<tf::Pose> ident (tf::Transform(tf::createIdentityQuaternion(),
                                             tf::Vector3(0,0,0)),
                                 ros::Time(), laser_scan->header.frame_id);
    tf::Stamped<tf::Pose> laser_pose;
    try
    {
      this->tf_->transformPose(base_frame_id_, ident, laser_pose);
    }
    catch(tf::TransformException& e)
    {
      ROS_ERROR("Couldn't transform from %s to %s, "
                "even though the message notifier is in use",
                laser_scan->header.frame_id.c_str(),
                base_frame_id_.c_str());
      return;
    }

    pf_vector_t laser_pose_v;
    laser_pose_v.v[0] = laser_pose.getOrigin().x();
    laser_pose_v.v[1] = laser_pose.getOrigin().y();
    // laser mounting angle gets computed later -> set to 0 here!
    laser_pose_v.v[2] = 0;
    lasers_[laser_index]->SetLaserPose(laser_pose_v);
    ROS_DEBUG("Received laser's pose wrt robot: %.3f %.3f %.3f",
              laser_pose_v.v[0],
              laser_pose_v.v[1],
              laser_pose_v.v[2]);

    frame_to_laser_[laser_scan->header.frame_id] = laser_index;
  } else {
    // we have the laser pose, retrieve laser index
    laser_index = frame_to_laser_[laser_scan->header.frame_id];
  }

  // Where was the robot when this scan was taken?
  pf_vector_t pose;
  if(!getOdomPose(latest_odom_pose_, pose.v[0], pose.v[1], pose.v[2],
                  laser_scan->header.stamp, base_frame_id_))
  {
    ROS_ERROR("Couldn't determine robot's pose associated with laser scan");
    return;
  }


  pf_vector_t delta = pf_vector_zero();

  if(pf_init_)
  {
    // Compute change in pose
    //delta = pf_vector_coord_sub(pose, pf_odom_pose_);
    delta.v[0] = pose.v[0] - pf_odom_pose_.v[0];
    delta.v[1] = pose.v[1] - pf_odom_pose_.v[1];
    delta.v[2] = angle_diff(pose.v[2], pf_odom_pose_.v[2]);

    // See if we should update the filter
    bool update;
    if (odom_integrator_topic_.size()) {
      double abs_trans = sqrt(odom_integrator_absolute_motion_.v[0]*odom_integrator_absolute_motion_.v[0] +
                              odom_integrator_absolute_motion_.v[1]*odom_integrator_absolute_motion_.v[1]);
      double abs_rot = odom_integrator_absolute_motion_.v[2];
      update = abs_trans >= d_thresh_ || abs_rot >= a_thresh_;
    } else {
      update = fabs(delta.v[0]) > d_thresh_ ||
               fabs(delta.v[1]) > d_thresh_ ||
               fabs(delta.v[2]) > a_thresh_;
    }
    update = update || m_force_update;
    m_force_update=false;

    // Set the laser update flags
    if(update)
      for(unsigned int i=0; i < lasers_update_.size(); i++)
        lasers_update_[i] = true;
  }

  bool force_publication = false;
  if(!pf_init_)
  {
    // Pose at last filter update
    pf_odom_pose_ = pose;

    // Filter is now initialized
    pf_init_ = true;

    // Should update sensor data
    for(unsigned int i=0; i < lasers_update_.size(); i++)
      lasers_update_[i] = true;

    force_publication = true;

    resample_count_ = 0;

    initOdomIntegrator();
  }
  // If the robot has moved, update the filter
  else if(pf_init_ && lasers_update_[laser_index])
  {
    //printf("pose\n");
    //pf_vector_fprintf(pose, stdout, "%.3f");

    AMCLOdomData odata;
    odata.pose = pose;
    // HACK
    // Modify the delta in the action data so the filter gets
    // updated correctly
    odata.delta = delta;
    odata.absolute_motion = odom_integrator_absolute_motion_;
    if (odom_integrator_topic_.size())
    {
      geometry_msgs::Pose2D p;
      p.x = odata.absolute_motion.v[0];
      p.y = odata.absolute_motion.v[1];
      p.theta = odata.absolute_motion.v[2];
      absolute_motion_pub_.publish(p);
    }

    // Use the action data to update the filter
    odom_->UpdateAction(pf_, (AMCLSensorData*)&odata);

    // Pose at last filter update
    //this->pf_odom_pose = pose;

    resetOdomIntegrator();
  }

  bool resampled = false;
  // If the robot has moved, update the filter
  if(lasers_update_[laser_index])
  {
    delete last_laser_data_;
    last_laser_data_ = new AMCLLaserData;
    AMCLLaserData &ldata = *last_laser_data_;
    ldata.sensor = lasers_[laser_index];
    ldata.range_count = laser_scan->ranges.size();

    // To account for lasers that are mounted upside-down, we determine the
    // min, max, and increment angles of the laser in the base frame.
    //
    // Construct min and max angles of laser, in the base_link frame.
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, laser_scan->angle_min);
    tf::Stamped<tf::Quaternion> min_q(q, laser_scan->header.stamp,
                                      laser_scan->header.frame_id);
    q.setRPY(0.0, 0.0, laser_scan->angle_min + laser_scan->angle_increment);
    tf::Stamped<tf::Quaternion> inc_q(q, laser_scan->header.stamp,
                                      laser_scan->header.frame_id);
    try
    {
      tf_->transformQuaternion(base_frame_id_, min_q, min_q);
      tf_->transformQuaternion(base_frame_id_, inc_q, inc_q);
    }
    catch(tf::TransformException& e)
    {
      ROS_WARN("Unable to transform min/max laser angles into base frame: %s",
               e.what());
      return;
    }

    double angle_min = tf::getYaw(min_q);
    double angle_increment = tf::getYaw(inc_q) - angle_min;

    // wrapping angle to [-pi .. pi]
    angle_increment = fmod(angle_increment + 5*M_PI, 2*M_PI) - M_PI;

    ROS_DEBUG("Laser %d angles in base frame: min: %.3f inc: %.3f", laser_index, angle_min, angle_increment);

    // Apply range min/max thresholds, if the user supplied them
    if(laser_max_range_ > 0.0)
      ldata.range_max = std::min(laser_scan->range_max, (float)laser_max_range_);
    else
      ldata.range_max = laser_scan->range_max;
    double range_min;
    if(laser_min_range_ > 0.0)
      range_min = std::max(laser_scan->range_min, (float)laser_min_range_);
    else
      range_min = laser_scan->range_min;
    // The AMCLLaserData destructor will free this memory
    ldata.ranges = new double[ldata.range_count][2];
    ROS_ASSERT(ldata.ranges);
    for(int i=0;i<ldata.range_count;i++)
    {
      // amcl doesn't (yet) have a concept of min range.  So we'll map short
      // readings to max range.
      if(laser_scan->ranges[i] <= range_min)
        ldata.ranges[i][0] = ldata.range_max;
      else
        ldata.ranges[i][0] = laser_scan->ranges[i];
      // Compute bearing
      ldata.ranges[i][1] = angle_min +
              (i * angle_increment);
    }

    lasers_[laser_index]->UpdateSensor(pf_, (AMCLSensorData*)&ldata);

    lasers_update_[laser_index] = false;

    pf_odom_pose_ = pose;

    // Resample the particles
    if(!(++resample_count_ % resample_interval_))
    {
      pf_update_resample(pf_);
      resampled = true;
      if(pf_->converged && global_localization_active_)
      {
        ROS_INFO("Global localization converged!");
        global_localization_active_ = false;
      }
    }

    pf_sample_set_t* set = pf_->sets + pf_->current_set;
    ROS_DEBUG("Num samples: %d\n", set->sample_count);

    // Publish the resulting cloud
    // TODO: set maximum rate for publishing
    if (!m_force_update) {
      geometry_msgs::PoseArray cloud_msg;
      cloud_msg.header.stamp = ros::Time::now();
      cloud_msg.header.frame_id = global_frame_id_;
      cloud_msg.poses.resize(set->sample_count);
      for(int i=0;i<set->sample_count;i++)
      {
        tf::poseTFToMsg(tf::Pose(tf::createQuaternionFromYaw(set->samples[i].pose.v[2]),
                                 tf::Vector3(set->samples[i].pose.v[0],
                                           set->samples[i].pose.v[1], 0)),
                        cloud_msg.poses[i]);
      }
      particlecloud_pub_.publish(cloud_msg);
      if (global_alt_frame_id_.size() > 0)
      {
        geometry_msgs::PoseArray alt_cloud_msg(cloud_msg);
        alt_cloud_msg.header.frame_id = global_alt_frame_id_;
        alt_particlecloud_pub_.publish(alt_cloud_msg);
      }
    }
  }

  if(resampled || force_publication)
  {
    // Read out the current hypotheses
    double max_weight = 0.0;
    int max_weight_hyp = -1;
    std::vector<amcl_hyp_t> hyps;
    hyps.resize(pf_->sets[pf_->current_set].cluster_count);
    for(int hyp_count = 0;
        hyp_count < pf_->sets[pf_->current_set].cluster_count; hyp_count++)
    {
      double weight;
      pf_vector_t pose_mean;
      pf_matrix_t pose_cov;
      if (!pf_get_cluster_stats(pf_, hyp_count, &weight, &pose_mean, &pose_cov))
      {
        ROS_ERROR("Couldn't get stats on cluster %d", hyp_count);
        break;
      }

      hyps[hyp_count].weight = weight;
      hyps[hyp_count].pf_pose_mean = pose_mean;
      hyps[hyp_count].pf_pose_cov = pose_cov;

      if(hyps[hyp_count].weight > max_weight)
      {
        max_weight = hyps[hyp_count].weight;
        max_weight_hyp = hyp_count;
      }
    }

    if(max_weight > 0.0)
    {
      ROS_DEBUG("Max weight pose: %.3f %.3f %.3f",
                hyps[max_weight_hyp].pf_pose_mean.v[0],
                hyps[max_weight_hyp].pf_pose_mean.v[1],
                hyps[max_weight_hyp].pf_pose_mean.v[2]);

      /*
         puts("");
         pf_matrix_fprintf(hyps[max_weight_hyp].pf_pose_cov, stdout, "%6.3f");
         puts("");
       */

      geometry_msgs::PoseWithCovarianceStamped p;
      // Fill in the header
      p.header.frame_id = global_frame_id_;
      p.header.stamp = laser_scan->header.stamp;
      // Copy in the pose
      p.pose.pose.position.x = hyps[max_weight_hyp].pf_pose_mean.v[0];
      p.pose.pose.position.y = hyps[max_weight_hyp].pf_pose_mean.v[1];
      tf::quaternionTFToMsg(tf::createQuaternionFromYaw(hyps[max_weight_hyp].pf_pose_mean.v[2]),
                            p.pose.pose.orientation);
      // Copy in the covariance, converting from 3-D to 6-D
      pf_sample_set_t* set = pf_->sets + pf_->current_set;
      for(int i=0; i<2; i++)
      {
        for(int j=0; j<2; j++)
        {
          // Report the overall filter covariance, rather than the
          // covariance for the highest-weight cluster
          //p.covariance[6*i+j] = hyps[max_weight_hyp].pf_pose_cov.m[i][j];
          p.pose.covariance[6*i+j] = set->cov.m[i][j];
        }
      }
      // Report the overall filter covariance, rather than the
      // covariance for the highest-weight cluster
      //p.covariance[6*5+5] = hyps[max_weight_hyp].pf_pose_cov.m[2][2];
      p.pose.covariance[6*5+5] = set->cov.m[2][2];

      /*
         printf("cov:\n");
         for(int i=0; i<6; i++)
         {
         for(int j=0; j<6; j++)
         printf("%6.3f ", p.covariance[6*i+j]);
         puts("");
         }
       */

      pose_pub_.publish(p);
      last_published_pose = p;
      if (global_alt_frame_id_.size() > 0)
      {
        geometry_msgs::PoseWithCovarianceStamped alt_p(p);
        alt_p.header.frame_id = global_alt_frame_id_;
        alt_pose_pub_.publish(alt_p);
      }

      ROS_DEBUG("New pose: %6.3f %6.3f %6.3f",
               hyps[max_weight_hyp].pf_pose_mean.v[0],
               hyps[max_weight_hyp].pf_pose_mean.v[1],
               hyps[max_weight_hyp].pf_pose_mean.v[2]);

      // subtracting base to odom from map to base and send map to odom instead
      tf::Stamped<tf::Pose> odom_to_map;
      try
      {
        tf::Transform tmp_tf(tf::createQuaternionFromYaw(hyps[max_weight_hyp].pf_pose_mean.v[2]),
                             tf::Vector3(hyps[max_weight_hyp].pf_pose_mean.v[0],
                                         hyps[max_weight_hyp].pf_pose_mean.v[1],
                                         0.0));
        tf::Stamped<tf::Pose> tmp_tf_stamped (tmp_tf.inverse(),
                                              laser_scan->header.stamp,
                                              base_frame_id_);
        this->tf_->transformPose(odom_frame_id_,
                                 tmp_tf_stamped,
                                 odom_to_map);
      }
      catch(tf::TransformException)
      {
        ROS_DEBUG("Failed to subtract base to odom transform");
        return;
      }

      boost::recursive_mutex::scoped_lock tfl(tf_mutex_);
      latest_tf_ = tf::Transform(tf::Quaternion(odom_to_map.getRotation()),
                                 tf::Point(odom_to_map.getOrigin()));
      latest_tf_valid_ = true;
    }
    else
    {
      ROS_ERROR("No pose!");
    }
  }
  else if(latest_tf_valid_)
  {

    // Is it time to save our last pose to the param server
    ros::Time now = ros::Time::now();
    if((save_pose_to_server_period.toSec() > 0.0) &&
       (now - save_pose_to_server_last_time) >= save_pose_to_server_period)
    {
      this->savePoseToServer();
      save_pose_to_server_last_time = now;
    }
    if((save_pose_to_file_period.toSec() > 0.0) &&
       (now - save_pose_to_file_last_time) >= save_pose_to_file_period)
    {
      ROS_DEBUG("save pose to file period: %f", save_pose_to_file_period.toSec());
      this->savePoseToFile();
      save_pose_to_file_last_time = now;
    }
  }
}

void
AmclNode::globalLocalizationCallback2D()
{
  laser_->SetMapFactors(global_localization_laser_off_map_factor_,
                        global_localization_laser_non_free_space_factor_,
                        laser_non_free_space_radius_);
  for (auto& l : lasers_)
  {
    l->SetMapFactors(global_localization_laser_off_map_factor_,
                     global_localization_laser_non_free_space_factor_,
                     laser_non_free_space_radius_);
  }
}

void
AmclNode::freeMapDependentMemory2D()
{
  delete laser_;
  laser_ = NULL;
}

void
AmclNode::deleteAmclNode2D()
{
  delete laser_scan_filter_;
  delete laser_scan_sub_;
}

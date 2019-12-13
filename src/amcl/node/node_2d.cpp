/*
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
// Desc: AMCL Node for 3D AMCL
// Author: Tyler Buchman (tyler_buchman@jabil.com)
//
///////////////////////////////////////////////////////////////////////////


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
#include "particle_filter.h"
#include "odom.h"
#include "planar_scanner.h"
#include "node.h"

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
Node::init2D()
{
  occupancy_map_ = NULL;
  planar_scanner_ = NULL;
  last_planar_data_ = NULL;
  private_nh_.param("planar_scanner_min_range", sensor_min_range_, -1.0);
  private_nh_.param("planar_scanner_max_range", sensor_max_range_, -1.0);
  private_nh_.param("planar_scanner_max_beams", max_beams_, 30);
  private_nh_.param("planar_scanner_z_hit", z_hit_, 0.95);
  private_nh_.param("planar_scanner_z_short", z_short_, 0.1);
  private_nh_.param("planar_scanner_z_max", z_max_, 0.05);
  private_nh_.param("planar_scanner_z_rand", z_rand_, 0.05);
  private_nh_.param("planar_scanner_sigma_hit", sigma_hit_, 0.2);
  private_nh_.param("planar_scanner_lambda_short", lambda_short_, 0.1);
  private_nh_.param("planar_scanner_likelihood_max_dist", sensor_likelihood_max_dist_, 2.0);
  private_nh_.param("planar_gompertz_a", planar_gompertz_a_, 1.0);
  private_nh_.param("planar_gompertz_b", planar_gompertz_b_, 1.0);
  private_nh_.param("planar_gompertz_c", planar_gompertz_c_, 1.0);
  private_nh_.param("planar_gompertz_input_shift", planar_gompertz_input_shift_, 0.0);
  private_nh_.param("planar_gompertz_input_scale", planar_gompertz_input_scale_, 1.0);
  private_nh_.param("planar_gompertz_output_shift", planar_gompertz_output_shift_, 0.0);
  private_nh_.param("planar_scanner_off_map_factor", off_map_factor_, 1.0);
  private_nh_.param("planar_scanner_non_free_space_factor", non_free_space_factor_, 1.0);
  private_nh_.param("planar_scanner_non_free_space_radius", non_free_space_radius_, 0.0);
  private_nh_.param("global_localization_planar_off_map_factor",
                    global_localization_off_map_factor_, 1.0);
  private_nh_.param("global_localization_planar_non_free_space_factor",
                    global_localization_non_free_space_factor_, 1.0);
  std::string tmp_model_type;
  private_nh_.param("planar_model_type", tmp_model_type, std::string("likelihood_field"));
  if(tmp_model_type == "beam")
    planar_model_type_ = PLANAR_MODEL_BEAM;
  else if(tmp_model_type == "likelihood_field")
    planar_model_type_ = PLANAR_MODEL_LIKELIHOOD_FIELD;
  else if(tmp_model_type == "likelihood_field_prob")
    planar_model_type_ = PLANAR_MODEL_LIKELIHOOD_FIELD_PROB;
  else if(tmp_model_type == "likelihood_field_gompertz")
    planar_model_type_ = PLANAR_MODEL_LIKELIHOOD_FIELD_GOMPERTZ;
  else
  {
    ROS_WARN("Unknown planar model type \"%s\"; defaulting to likelihood_field model",
             tmp_model_type.c_str());
    planar_model_type_ = PLANAR_MODEL_LIKELIHOOD_FIELD;
  }
  private_nh_.param("map_scale_up_factor", map_scale_up_factor_, 1);
  // Prevent nonsense and crashes due to wacky values
  if (map_scale_up_factor_ < 1)
    map_scale_up_factor_ = 1;
  if (map_scale_up_factor_ > 16)
    map_scale_up_factor_ = 16;

  if(map_type_ == 2)
  {
    planar_scan_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, planar_scan_topic_, 100);
    planar_scan_filter_ =
        new tf::MessageFilter<sensor_msgs::LaserScan>(*planar_scan_sub_, *tf_, odom_frame_id_, 100);
    planar_scan_filter_->registerCallback(boost::bind(&Node::planarScanReceived, this, _1));

    // 15s timer to warn on lack of receipt of planar scans, #5209
    planar_scanner_check_interval_ = ros::Duration(15.0);
    check_planar_scanner_timer_ = nh_.createTimer(planar_scanner_check_interval_,
                                       boost::bind(&Node::checkPlanarScanReceived, this, _1));
  }
}

void
Node::checkPlanarScanReceived(const ros::TimerEvent& event)
{
  ros::Duration d = ros::Time::now() - last_planar_scan_received_ts_;
  if(d > planar_scanner_check_interval_)
  {
    ROS_WARN("No planar scan received (and thus no pose updates have been published) for %f seconds."
             "Verify that data is being published on the %s topic.",
             d.toSec(), ros::names::resolve(planar_scan_topic_).c_str());
  }
}

/**
 * Convert an OccupancyGrid map message into the internal
 * representation.  This allocates an OccupancyMap and returns it.
 */
OccupancyMap*
Node::convertMap( const nav_msgs::OccupancyGrid& map_msg )
{
  OccupancyMap* occupancy_map = new OccupancyMap();
  ROS_ASSERT(occupancy_map);
  std::vector<int> size_vec;
  double scale = map_msg.info.resolution / map_scale_up_factor_;
  size_vec.push_back(map_msg.info.width * map_scale_up_factor_);
  size_vec.push_back(map_msg.info.height * map_scale_up_factor_);
  occupancy_map->setSize(size_vec);
  occupancy_map->setScale(scale);
  std::vector<double> origin;
  origin.push_back(map_msg.info.origin.position.x + (size_vec[0] / 2) * scale);
  origin.push_back(map_msg.info.origin.position.y + (size_vec[1] / 2) * scale);
  occupancy_map->setOrigin(origin);
  occupancy_map->initCells(size_vec[0]*size_vec[1]);
  ROS_ASSERT(occupancy_map->getCells());
  for(int y=0;y<size_vec[1];y++)
  {
    int i=y*size_vec[0];
    const int msg_row=(y/map_scale_up_factor_)*(map_msg.info.width);
    for(int x=0;x<size_vec[0];x++,i++)
    {
      const int msg_i=msg_row+x/map_scale_up_factor_;

      if(map_msg.data[msg_i] == 0)
        occupancy_map->setCellOccState(i, -1);
      else if(map_msg.data[msg_i] == 100)
        occupancy_map->setCellOccState(i, +1);
      else
        occupancy_map->setCellOccState(i, 0);
    }
  }
  return occupancy_map;
}

// Helper function to score a pose for uniform pose generation
double
Node::scorePose2D(const PFVector &p)
{
  if(this->last_planar_data_ == NULL)
  {
    // There is no data to match, so return a perfect match
    return 1.0;
  }
  // Create a fake "sample set" of just this pose to score it.
  PFSample fake_sample;
  fake_sample.pose.v[0] = p.v[0];
  fake_sample.pose.v[1] = p.v[1];
  fake_sample.pose.v[2] = p.v[2];
  fake_sample.weight = 1.0;
  PFSampleSet fake_sample_set;
  fake_sample_set.sample_count = 1;
  fake_sample_set.samples = &fake_sample;
  fake_sample_set.converged = 0;
  PlanarScanner::applyModelToSampleSet(this->last_planar_data_, &fake_sample_set);
  return fake_sample.weight;
}

void
Node::reconfigure2D(AMCLConfig &config)
{
  sensor_min_range_ = config.planar_scanner_min_range;
  sensor_max_range_ = config.planar_scanner_max_range;
  z_hit_ = config.planar_scanner_z_hit;
  z_short_ = config.planar_scanner_z_short;
  z_max_ = config.planar_scanner_z_max;
  z_rand_ = config.planar_scanner_z_rand;
  sigma_hit_ = config.planar_scanner_sigma_hit;
  lambda_short_ = config.planar_scanner_lambda_short;
  sensor_likelihood_max_dist_ = config.planar_scanner_likelihood_max_dist;
  off_map_factor_ = config.planar_scanner_off_map_factor;
  non_free_space_factor_ = config.planar_scanner_non_free_space_factor;
  non_free_space_radius_ = config.planar_scanner_non_free_space_radius;
  global_localization_off_map_factor_ = config.global_localization_planar_scanner_off_map_factor;
  global_localization_non_free_space_factor_ = config.global_localization_planar_scanner_non_free_space_factor;

  planar_gompertz_a_ = config.planar_gompertz_a;
  planar_gompertz_b_ = config.planar_gompertz_b;
  planar_gompertz_c_ = config.planar_gompertz_c;
  planar_gompertz_input_shift_ = config.planar_gompertz_input_shift;
  planar_gompertz_input_scale_ = config.planar_gompertz_input_scale;
  planar_gompertz_output_shift_ = config.planar_gompertz_output_shift;
  max_beams_ = config.planar_scanner_max_beams;
  if(config.planar_model_type == "beam")
    planar_model_type_ = PLANAR_MODEL_BEAM;
  else if(config.planar_model_type == "likelihood_field")
    planar_model_type_ = PLANAR_MODEL_LIKELIHOOD_FIELD;
  else if(config.planar_model_type == "likelihood_field_prob")
    planar_model_type_ = PLANAR_MODEL_LIKELIHOOD_FIELD_PROB;
  else if(config.planar_model_type == "likelihood_field_gompertz")
    planar_model_type_ = PLANAR_MODEL_LIKELIHOOD_FIELD_GOMPERTZ;
  delete planar_scanner_;
  planar_scanner_ = new PlanarScanner(max_beams_, occupancy_map_);
  ROS_ASSERT(planar_scanner_);
  if(planar_model_type_ == PLANAR_MODEL_BEAM)
    planar_scanner_->setModelBeam(z_hit_, z_short_, z_max_, z_rand_,
                         sigma_hit_, lambda_short_);
  else if(planar_model_type_ == PLANAR_MODEL_LIKELIHOOD_FIELD_PROB){
    ROS_INFO("Initializing likelihood field model; this can take some time on large maps...");
    planar_scanner_->setModelLikelihoodFieldProb(z_hit_, z_rand_, sigma_hit_,
					sensor_likelihood_max_dist_,
					do_beamskip_, beam_skip_distance_,
					beam_skip_threshold_, beam_skip_error_threshold_);
    ROS_INFO("Done initializing likelihood field model with probabilities.");
  }
  else if(planar_model_type_ == PLANAR_MODEL_LIKELIHOOD_FIELD){
    ROS_INFO("Initializing likelihood field model; this can take some time on large maps...");
    planar_scanner_->setModelLikelihoodField(z_hit_, z_rand_, sigma_hit_,
                                    sensor_likelihood_max_dist_);
    ROS_INFO("Done initializing likelihood field model.");
  }
  else if(planar_model_type_ == PLANAR_MODEL_LIKELIHOOD_FIELD_GOMPERTZ){
    ROS_INFO("Initializing likelihood field (gompertz) model; this can take some time on large maps...");
    planar_scanner_->setModelLikelihoodFieldGompertz(z_hit_, z_rand_, sigma_hit_,
                                            sensor_likelihood_max_dist_,
                                            planar_gompertz_a_,
                                            planar_gompertz_b_,
                                            planar_gompertz_c_,
                                            planar_gompertz_input_shift_,
                                            planar_gompertz_input_scale_,
                                            planar_gompertz_output_shift_);
    ROS_INFO("Gompertz key points by total planar scan match: "
        "0.0: %f, 0.25: %f, 0.5: %f, 0.75: %f, 1.0: %f",
        planar_scanner_->applyGompertz(z_rand_),
        planar_scanner_->applyGompertz(z_rand_ + z_hit_ * .25),
        planar_scanner_->applyGompertz(z_rand_ + z_hit_ * .5),
        planar_scanner_->applyGompertz(z_rand_ + z_hit_ * .75),
        planar_scanner_->applyGompertz(z_rand_ + z_hit_));
    ROS_INFO("Done initializing likelihood (gompertz) field model.");
  }
  planar_scanner_->setMapFactors(off_map_factor_, non_free_space_factor_, non_free_space_radius_);
  delete planar_scan_filter_;
  planar_scan_filter_ =
          new tf::MessageFilter<sensor_msgs::LaserScan>(*planar_scan_sub_,
                                                        *tf_,
                                                        odom_frame_id_,
                                                        100);
  planar_scan_filter_->registerCallback(boost::bind(&Node::planarScanReceived,
                                                   this, _1));
}

void
Node::occupancyMapMsgReceived(const nav_msgs::OccupancyGridConstPtr& msg)
{
  if( first_map_only_ && first_occupancy_map_received_ ) {
    ROS_DEBUG("occupancy map already received");
    return;
  }

  boost::recursive_mutex::scoped_lock cfl(configuration_mutex_);

  ROS_INFO("Received a %d X %d occupancy map @ %.3f m/pix\n",
           msg->info.width,
           msg->info.height,
           msg->info.resolution);

  delete occupancy_map_;
  occupancy_map_ = NULL;

  occupancy_map_ = convertMap(*msg);
  first_occupancy_map_received_ = true;

  // if the OccupancyMap is a secondary map source
  if(map_type_ == 3)
  {
    if(first_octomap_received_ and wait_for_occupancy_map_)
    {
      std::vector<double> map_min, map_max(2);
      map_min = {0.0, 0.0};
      occupancy_map_->convertMapToWorld(occupancy_map_->getSize(), &map_max);
      octomap_->setMapBounds(map_min, map_max);
      update3DFreeSpaceIndices();
    }
    return;
  }
  else if(map_type_ != 2)
  {
    return;
  }

  delete map_;
  map_ = NULL;

  map_ = occupancy_map_;
  freeMapDependentMemory();
  // Clear queued planar scanner objects because they hold pointers to the existing map
  planar_scanners_.clear();
  planar_scanners_update_.clear();
  frame_to_planar_scanner_.clear();
  delete last_planar_data_;
  last_planar_data_ = NULL;
  initFromNewMap();
}

void
Node::initFromNewOccupancyMap()
{
  delete planar_scanner_;
  planar_scanner_ = new PlanarScanner(max_beams_, occupancy_map_);
  ROS_ASSERT(planar_scanner_);
  if(planar_model_type_ == PLANAR_MODEL_BEAM)
    planar_scanner_->setModelBeam(z_hit_, z_short_, z_max_, z_rand_,
                         sigma_hit_, lambda_short_);
  else if(planar_model_type_ == PLANAR_MODEL_LIKELIHOOD_FIELD_PROB){
    ROS_INFO("Initializing likelihood field model; this can take some time on large maps...");
    planar_scanner_->setModelLikelihoodFieldProb(z_hit_, z_rand_, sigma_hit_,
					sensor_likelihood_max_dist_,
					do_beamskip_, beam_skip_distance_,
					beam_skip_threshold_, beam_skip_error_threshold_);
    ROS_INFO("Done initializing likelihood field model.");
  }
  else if(planar_model_type_ == PLANAR_MODEL_LIKELIHOOD_FIELD_GOMPERTZ){
    ROS_INFO("Initializing likelihood field (gompertz) model; this can take some time on large maps...");
    planar_scanner_->setModelLikelihoodFieldGompertz(z_hit_, z_rand_, sigma_hit_,
                                            sensor_likelihood_max_dist_,
                                            planar_gompertz_a_,
                                            planar_gompertz_b_,
                                            planar_gompertz_c_,
                                            planar_gompertz_input_shift_,
                                            planar_gompertz_input_scale_,
                                            planar_gompertz_output_shift_);
    ROS_INFO("Gompertz key points by total planar scan match: "
        "0.0: %f, 0.25: %f, 0.5: %f, 0.75: %f, 1.0: %f",
        planar_scanner_->applyGompertz(z_rand_),
        planar_scanner_->applyGompertz(z_rand_ + z_hit_ * .25),
        planar_scanner_->applyGompertz(z_rand_ + z_hit_ * .5),
        planar_scanner_->applyGompertz(z_rand_ + z_hit_ * .75),
        planar_scanner_->applyGompertz(z_rand_ + z_hit_));
    ROS_INFO("Done initializing likelihood (gompertz) field model.");
  }
  else
  {
    ROS_INFO("Initializing likelihood field model; this can take some time on large maps...");
    planar_scanner_->setModelLikelihoodField(z_hit_, z_rand_, sigma_hit_,
                                    sensor_likelihood_max_dist_);
    ROS_INFO("Done initializing likelihood field model.");
  }
  planar_scanner_->setMapFactors(off_map_factor_, non_free_space_factor_, non_free_space_radius_);

  // Index of free space
  // Must be calculated after the occ_dist is setup by the planar model
  free_space_indices_.resize(0);
  std::vector<int> size_vec = map_->getSize();
  for(int i = 0; i < size_vec[0]; i++)
    for(int j = 0; j < size_vec[1]; j++)
      if(((OccupancyMap*)map_)->getOccState(i,j) == -1)
        if(((OccupancyMap*)map_)->getOccDist(i,j) > non_free_space_radius_)
          free_space_indices_.push_back(std::make_pair(i,j));
}

void
Node::planarScanReceived(const sensor_msgs::LaserScanConstPtr& planar_scan)
{
  last_planar_scan_received_ts_ = ros::Time::now();
  if(map_ == NULL) {
    return;
  }
  if(not occupancy_map_->isCSpaceCreated())
  {
    ROS_DEBUG("CSpace not yet created");
    return;
  }

  boost::recursive_mutex::scoped_lock cfl(configuration_mutex_);
  int scanner_index = -1;

  // Handle corner cases like getting dynamically reconfigured or getting a
  // new map by de-activating the global localization parameters here if we are
  // no longer globally localizing.
  if(!global_localization_active_)
  {
    pf_->setDecayRates(alpha_slow_, alpha_fast_);
    planar_scanner_->setMapFactors(off_map_factor_, non_free_space_factor_, non_free_space_radius_);
    for (auto& l : planar_scanners_)
    {
      l->setMapFactors(off_map_factor_, non_free_space_factor_, non_free_space_radius_);
    }
  }

  // Do we have the base->base_laser Tx yet?
  if(frame_to_planar_scanner_.find(planar_scan->header.frame_id) == frame_to_planar_scanner_.end())
  {
    ROS_DEBUG("Setting up planar_scanner %d (frame_id=%s)\n", (int)frame_to_planar_scanner_.size(), planar_scan->header.frame_id.c_str());
    planar_scanners_.push_back(new PlanarScanner(*planar_scanner_));
    planar_scanners_update_.push_back(true);
    scanner_index = frame_to_planar_scanner_.size();

    tf::Stamped<tf::Pose> ident (tf::Transform(tf::createIdentityQuaternion(),
                                             tf::Vector3(0,0,0)),
                                 ros::Time(), planar_scan->header.frame_id);
    tf::Stamped<tf::Pose> planar_scanner_pose;
    try
    {
      this->tf_->transformPose(base_frame_id_, ident, planar_scanner_pose);
    }
    catch(tf::TransformException& e)
    {
      ROS_ERROR("Couldn't transform from %s to %s, "
                "even though the message notifier is in use",
                planar_scan->header.frame_id.c_str(),
                base_frame_id_.c_str());
      return;
    }

    PFVector planar_scanner_pose_v;
    planar_scanner_pose_v.v[0] = planar_scanner_pose.getOrigin().x();
    planar_scanner_pose_v.v[1] = planar_scanner_pose.getOrigin().y();
    // planar scanner mounting angle gets computed later -> set to 0 here!
    planar_scanner_pose_v.v[2] = 0;
    planar_scanners_[scanner_index]->setPlanarScannerPose(planar_scanner_pose_v);
    ROS_DEBUG("Received planar scanner's pose wrt robot: %.3f %.3f %.3f",
              planar_scanner_pose_v.v[0],
              planar_scanner_pose_v.v[1],
              planar_scanner_pose_v.v[2]);

    frame_to_planar_scanner_[planar_scan->header.frame_id] = scanner_index;
  } else {
    // we have the planar scanner pose, retrieve planar scanner index
    scanner_index = frame_to_planar_scanner_[planar_scan->header.frame_id];
  }

  // Where was the robot when this scan was taken?
  PFVector pose;
  if(!getOdomPose(planar_scan->header.stamp, base_frame_id_, &latest_odom_pose_, &pose))
  {
    ROS_ERROR("Couldn't determine robot's pose associated with planar scan");
    return;
  }

  PFVector delta;

  if(pf_init_)
  {
    // Compute change in pose
    delta.v[0] = pose.v[0] - pf_odom_pose_.v[0];
    delta.v[1] = pose.v[1] - pf_odom_pose_.v[1];
    delta.v[2] = angleDiff(pose.v[2], pf_odom_pose_.v[2]);

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

    // Set the planar scanner update flags
    if(update)
      for(unsigned int i=0; i < planar_scanners_update_.size(); i++)
        planar_scanners_update_[i] = true;
  }

  bool force_publication = false;
  if(!pf_init_)
  {
    // Pose at last filter update
    pf_odom_pose_ = pose;

    // Filter is now initialized
    pf_init_ = true;

    // Should update sensor data
    for(unsigned int i=0; i < planar_scanners_update_.size(); i++)
      planar_scanners_update_[i] = true;

    force_publication = true;

    resample_count_ = 0;

    initOdomIntegrator();
  }
  // If the robot has moved, update the filter
  else if(pf_init_ && planar_scanners_update_[scanner_index])
  {
    OdomData odata;
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
    odom_->updateAction(pf_, (SensorData*)&odata);

    resetOdomIntegrator();
  }

  bool resampled = false;
  // If the robot has moved, update the filter
  if(planar_scanners_update_[scanner_index])
  {
    delete last_planar_data_;
    last_planar_data_ = new PlanarData;
    PlanarData &ldata = *last_planar_data_;
    ldata.sensor_ = planar_scanners_[scanner_index];
    ldata.range_count_ = planar_scan->ranges.size();

    // To account for the planar scanners that are mounted upside-down, we determine the
    // min, max, and increment angles of the scanner in the base frame.
    //
    // Construct min and max angles of scanner, in the base_link frame.
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, planar_scan->angle_min);
    tf::Stamped<tf::Quaternion> min_q(q, planar_scan->header.stamp,
                                      planar_scan->header.frame_id);
    q.setRPY(0.0, 0.0, planar_scan->angle_min + planar_scan->angle_increment);
    tf::Stamped<tf::Quaternion> inc_q(q, planar_scan->header.stamp,
                                      planar_scan->header.frame_id);
    try
    {
      tf_->transformQuaternion(base_frame_id_, min_q, min_q);
      tf_->transformQuaternion(base_frame_id_, inc_q, inc_q);
    }
    catch(tf::TransformException& e)
    {
      ROS_WARN("Unable to transform min/max planar scanner angles into base frame: %s",
               e.what());
      return;
    }

    double angle_min = tf::getYaw(min_q);
    double angle_increment = tf::getYaw(inc_q) - angle_min;

    // wrapping angle to [-pi .. pi]
    angle_increment = fmod(angle_increment + 5*M_PI, 2*M_PI) - M_PI;

    ROS_DEBUG("Planar scanner %d angles in base frame: min: %.3f inc: %.3f", scanner_index, angle_min, angle_increment);

    // Apply range min/max thresholds, if the user supplied them
    if(sensor_max_range_ > 0.0)
      ldata.range_max_ = std::min(planar_scan->range_max, (float)sensor_max_range_);
    else
      ldata.range_max_ = planar_scan->range_max;
    double range_min;
    if(sensor_min_range_ > 0.0)
      range_min = std::max(planar_scan->range_min, (float)sensor_min_range_);
    else
      range_min = planar_scan->range_min;
    // The PlanarData destructor will free this memory
    ldata.ranges_ = new double[ldata.range_count_][2];
    ROS_ASSERT(ldata.ranges_);
    for(int i=0;i<ldata.range_count_;i++)
    {
      // amcl doesn't (yet) have a concept of min range.  So we'll map short
      // readings to max range.
      if(planar_scan->ranges[i] <= range_min)
        ldata.ranges_[i][0] = ldata.range_max_;
      else
        ldata.ranges_[i][0] = planar_scan->ranges[i];
      // Compute bearing
      ldata.ranges_[i][1] = angle_min +
              (i * angle_increment);
    }

    planar_scanners_[scanner_index]->updateSensor(pf_, (SensorData*)&ldata);

    planar_scanners_update_[scanner_index] = false;

    pf_odom_pose_ = pose;

    // Resample the particles
    if(++resample_count_ % resample_interval_ == 0)
    {
      pf_->updateResample();
      resampled = true;
      if(pf_->isConverged() && global_localization_active_)
      {
        ROS_INFO("Global localization converged!");
        global_localization_active_ = false;
      }
    }

    PFSampleSet* set = pf_->getCurrentSet();
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
                                 tf::Vector3(set->samples[i].pose.v[0], set->samples[i].pose.v[1], 0)
                                ), cloud_msg.poses[i]);
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
    std::vector<AMCLHyp> hyps;
    hyps.resize(pf_->getCurrentSet()->cluster_count);
    for(int hyp_count = 0;
        hyp_count < pf_->getCurrentSet()->cluster_count; hyp_count++)
    {
      double weight;
      PFVector pose_mean;
      PFMatrix pose_cov;
      if (!pf_->getClusterStats(hyp_count, &weight, &pose_mean, &pose_cov))
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

      geometry_msgs::PoseWithCovarianceStamped p;
      // Fill in the header
      p.header.frame_id = global_frame_id_;
      p.header.stamp = planar_scan->header.stamp;
      // Copy in the pose
      p.pose.pose.position.x = hyps[max_weight_hyp].pf_pose_mean.v[0];
      p.pose.pose.position.y = hyps[max_weight_hyp].pf_pose_mean.v[1];
      tf::quaternionTFToMsg(tf::createQuaternionFromYaw(hyps[max_weight_hyp].pf_pose_mean.v[2]),
                            p.pose.pose.orientation);
      // Copy in the covariance, converting from 3-D to 6-D
      PFSampleSet* set = pf_->getCurrentSet();
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

      pose_pub_.publish(p);
      last_published_pose_ = p;
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
                                              planar_scan->header.stamp,
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
    if((save_pose_to_server_period_.toSec() > 0.0) &&
       (now - save_pose_to_server_last_time_) >= save_pose_to_server_period_)
    {
      this->savePoseToServer();
      save_pose_to_server_last_time_ = now;
    }
    if((save_pose_to_file_period_.toSec() > 0.0) &&
       (now - save_pose_to_file_last_time_) >= save_pose_to_file_period_)
    {
      ROS_DEBUG("save pose to file period: %f", save_pose_to_file_period_.toSec());
      this->savePoseToFile();
      save_pose_to_file_last_time_ = now;
    }
  }
}

void
Node::globalLocalizationCallback2D()
{
  planar_scanner_->setMapFactors(global_localization_off_map_factor_,
                        global_localization_non_free_space_factor_,
                        non_free_space_radius_);
  for (auto& l : planar_scanners_)
  {
    l->setMapFactors(global_localization_off_map_factor_,
                     global_localization_non_free_space_factor_,
                     non_free_space_radius_);
  }
}

void
Node::freeOccupancyMapDependentMemory()
{
  delete planar_scanner_;
  planar_scanner_ = NULL;
}

void
Node::deleteNode2D()
{
  delete planar_scan_filter_;
  delete planar_scan_sub_;
}

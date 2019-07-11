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
#include "pf.h"
#include "amcl_odom.h"
#include "amcl_node.h"

#include "ros/assert.h"

// roscpp
#include "ros/ros.h"

// Messages that I need
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

AmclNode::AmclNode() :
        sent_first_transform_(false),
        latest_tf_valid_(false),
        map_(NULL),
        pf_(NULL),
        resample_count_(0),
        odom_(NULL),
	    private_nh_("~"),
        initial_pose_hyp_(NULL),
        first_map_received_(false),
        first_reconfigure_call_(true),
        global_localization_active_(false)
{
  boost::recursive_mutex::scoped_lock cfl(configuration_mutex_);

  // 2: 2d, 3: 3d, else: none
  private_nh_.param("map_type", map_type_, 0);
  // Grab params off the param server
  private_nh_.param("first_map_only", first_map_only_, false);
  private_nh_.param("map_scale_up_factor", map_scale_up_factor_, 1);
  // Prevent nonsense and crashes due to wacky values
  if (map_scale_up_factor_ < 1)
    map_scale_up_factor_ = 1;
  if (map_scale_up_factor_ > 16)
    map_scale_up_factor_ = 16;

  double tmp;
  private_nh_.param("gui_publish_rate", tmp, -1.0);
  gui_publish_period = ros::Duration(1.0/tmp);
  private_nh_.param("transform_publish_rate", tmp, 50.0);
  transform_publish_period = ros::Duration(1.0/tmp);
  private_nh_.param("save_pose_to_server_rate", tmp, 2.0);
  save_pose_to_server_period = ros::Duration(1.0/tmp);
  private_nh_.param("save_pose_to_file_rate", tmp, 0.1);
  save_pose_to_file_period = ros::Duration(1.0/tmp);

  private_nh_.param("min_particles", min_particles_, 100);
  private_nh_.param("max_particles", max_particles_, 5000);
  private_nh_.param("kld_err", pf_err_, 0.01);
  private_nh_.param("kld_z", pf_z_, 0.99);
  private_nh_.param("odom_alpha1", alpha1_, 0.2);
  private_nh_.param("odom_alpha2", alpha2_, 0.2);
  private_nh_.param("odom_alpha3", alpha3_, 0.2);
  private_nh_.param("odom_alpha4", alpha4_, 0.2);
  private_nh_.param("odom_alpha5", alpha5_, 0.2);

  private_nh_.param("do_beamskip", do_beamskip_, false);
  private_nh_.param("beam_skip_distance", beam_skip_distance_, 0.5);
  private_nh_.param("beam_skip_threshold", beam_skip_threshold_, 0.3);
  private_nh_.param("beam_skip_error_threshold_", beam_skip_error_threshold_, 0.9);

  private_nh_.param("save_pose", save_pose_, false);
  const std::string default_filename = "savedpose.yaml";
  std::string filename;
  private_nh_.param("saved_pose_filename", filename, default_filename);
  saved_pose_filepath_ = makeFilepathFromName(filename);

  std::string tmp_model_type;
  private_nh_.param("odom_model_type", tmp_model_type, std::string("diff"));
  if(tmp_model_type == "diff")
    odom_model_type_ = ODOM_MODEL_DIFF;
  else if(tmp_model_type == "omni")
    odom_model_type_ = ODOM_MODEL_OMNI;
  else if(tmp_model_type == "diff-corrected")
    odom_model_type_ = ODOM_MODEL_DIFF_CORRECTED;
  else if(tmp_model_type == "omni-corrected")
    odom_model_type_ = ODOM_MODEL_OMNI_CORRECTED;
  else if(tmp_model_type == "gaussian")
    odom_model_type_ = ODOM_MODEL_GAUSSIAN;
  else
  {
    ROS_WARN("Unknown odom model type \"%s\"; defaulting to diff model",
             tmp_model_type.c_str());
    odom_model_type_ = ODOM_MODEL_DIFF;
  }

  private_nh_.param("update_min_d", d_thresh_, 0.2);
  private_nh_.param("update_min_a", a_thresh_, M_PI/6.0);
  private_nh_.param("odom_frame_id", odom_frame_id_, std::string("odom"));
  private_nh_.param("base_frame_id", base_frame_id_, std::string("base_link"));
  private_nh_.param("global_frame_id", global_frame_id_, std::string("map"));
  private_nh_.param("global_alt_frame_id", global_alt_frame_id_, std::string(""));
  private_nh_.param("resample_interval", resample_interval_, 2);
  private_nh_.param("resample_model_type", tmp_model_type, std::string("multinomial"));
  if(tmp_model_type == "multinomial")
    resample_model_type_ = PF_RESAMPLE_MULTINOMIAL;
  else if(tmp_model_type == "systematic")
    resample_model_type_ = PF_RESAMPLE_SYSTEMATIC;
  else
  {
    ROS_WARN("Unknown resample model type \"%s\"; defaulting to multinomial model",
             tmp_model_type.c_str());
    resample_model_type_ = PF_RESAMPLE_MULTINOMIAL;
  }

  double tmp_tol;
  private_nh_.param("transform_tolerance", tmp_tol, 0.1);
  private_nh_.param("recovery_alpha_slow", alpha_slow_, 0.001);
  private_nh_.param("recovery_alpha_fast", alpha_fast_, 0.1);
  private_nh_.param("uniform_pose_starting_weight_threshold", uniform_pose_starting_weight_threshold_, 0.0);
  private_nh_.param("uniform_pose_deweight_multiplier", uniform_pose_deweight_multiplier_, 0.0);
  private_nh_.param("global_localization_alpha_slow", global_localization_alpha_slow_, 0.001);
  private_nh_.param("global_localization_alpha_fast", global_localization_alpha_fast_, 0.1);
  private_nh_.param("tf_broadcast", tf_broadcast_, true);
  private_nh_.param("tf_reverse", tf_reverse_, false);
  private_nh_.param("odom_integrator_topic", odom_integrator_topic_, std::string(""));

  transform_tolerance_.fromSec(tmp_tol);

  initial_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);

  cloud_pub_interval.fromSec(1.0);
  tfb_ = new tf::TransformBroadcaster();
  tf_ = new TransformListenerWrapper();

  pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 2, true);
  particlecloud_pub_ = nh_.advertise<geometry_msgs::PoseArray>("particlecloud", 2, true);
  if (global_alt_frame_id_.size() > 0)
  {
    alt_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose_in_"+global_alt_frame_id_, 2, true);
    alt_particlecloud_pub_ = nh_.advertise<geometry_msgs::PoseArray>("particlecloud_in_"+global_alt_frame_id_, 2, true);
  }
  map_odom_transform_pub_ = nh_.advertise<nav_msgs::Odometry>("amcl_map_odom_transform", 1);
  global_loc_srv_ = nh_.advertiseService("global_localization",
					 &AmclNode::globalLocalizationCallback,
                                         this);

  loadPose();

  if(map_type_ == 2)
  {
    init2D();
  }
  else if(map_type_ == 3)
  {
    init3D();
  }

  initial_pose_sub_ = nh_.subscribe("initialpose", 2, &AmclNode::initialPoseReceived, this);

  if (odom_integrator_topic_.size()) {
    odom_integrator_sub_ = nh_.subscribe(odom_integrator_topic_, 20, &AmclNode::integrateOdom, this);
    absolute_motion_pub_ = nh_.advertise<geometry_msgs::Pose2D>("amcl_absolute_motion", 20, false);
  }

  map_sub_ = nh_.subscribe("map", 1, &AmclNode::mapReceived, this);
  ROS_INFO("Subscribed to map topic.");

  m_force_update = false;

  dsrv_ = new dynamic_reconfigure::Server<amcl::AMCLConfig>(ros::NodeHandle("~"));
  dynamic_reconfigure::Server<amcl::AMCLConfig>::CallbackType cb = boost::bind(&AmclNode::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);

  publish_transform_interval_ = ros::Duration(transform_publish_period);
  publish_transform_timer_ = nh_.createTimer(publish_transform_interval_,
                                             boost::bind(&AmclNode::publishTransform, this, _1));
}

void
AmclNode::reconfigureCB(AMCLConfig &config, uint32_t level)
{
  boost::recursive_mutex::scoped_lock cfl(configuration_mutex_);

  //we don't want to do anything on the first call
  //which corresponds to startup
  if(first_reconfigure_call_)
  {
    first_reconfigure_call_ = false;
    default_config_ = config;
    return;
  }

  if(config.restore_defaults) {
    config = default_config_;
    //avoid looping
    config.restore_defaults = false;
  }

  // 2: 2d, 3: 3d, else: none
  map_type_ = config.map_type;

  d_thresh_ = config.update_min_d;
  a_thresh_ = config.update_min_a;

  resample_interval_ = config.resample_interval;
  if(config.resample_model_type == "multinomial")
    resample_model_type_ = PF_RESAMPLE_MULTINOMIAL;
  else if(config.resample_model_type == "systematic")
    resample_model_type_ = PF_RESAMPLE_SYSTEMATIC;
  else
  {
    ROS_WARN("Unknown resample model type \"%s\"; defaulting to multinomial model",
             config.resample_model_type.c_str());
    resample_model_type_ = PF_RESAMPLE_MULTINOMIAL;
  }

  gui_publish_period = ros::Duration(1.0/config.gui_publish_rate);
  transform_publish_period = ros::Duration(1.0/config.transform_publish_rate);
  save_pose_to_server_period = ros::Duration(1.0/config.save_pose_to_server_rate);
  save_pose_to_file_period = ros::Duration(1.0/config.save_pose_to_file_rate);

  transform_tolerance_.fromSec(config.transform_tolerance);

  alpha1_ = config.odom_alpha1;
  alpha2_ = config.odom_alpha2;
  alpha3_ = config.odom_alpha3;
  alpha4_ = config.odom_alpha4;
  alpha5_ = config.odom_alpha5;

  if(config.min_particles > config.max_particles)
  {
    ROS_WARN("You've set min_particles to be greater than max particles, this isn't allowed so they'll be set to be equal.");
    config.max_particles = config.min_particles;
  }

  min_particles_ = config.min_particles;
  max_particles_ = config.max_particles;
  alpha_slow_ = config.recovery_alpha_slow;
  alpha_fast_ = config.recovery_alpha_fast;
  uniform_pose_starting_weight_threshold_ = config.uniform_pose_starting_weight_threshold;
  uniform_pose_deweight_multiplier_ = config.uniform_pose_deweight_multiplier;
  global_localization_alpha_slow_ = config.global_localization_alpha_slow;
  global_localization_alpha_fast_ = config.global_localization_alpha_fast;
  tf_broadcast_ = config.tf_broadcast;
  tf_reverse_ = config.tf_reverse;

  do_beamskip_= config.do_beamskip;
  beam_skip_distance_ = config.beam_skip_distance;
  beam_skip_threshold_ = config.beam_skip_threshold;

  pf_ = pf_alloc(min_particles_, max_particles_,
                 alpha_slow_, alpha_fast_,
                 (pf_init_model_fn_t)AmclNode::uniformPoseGenerator,
                 (void *)this);
  pf_err_ = config.kld_err;
  pf_z_ = config.kld_z;
  pf_->pop_err = pf_err_;
  pf_->pop_z = pf_z_;
  pf_set_resample_model(pf_, resample_model_type_);

  // Initialize the filter
  pf_vector_t pf_init_pose_mean = pf_vector_zero();
  pf_init_pose_mean.v[0] = last_published_pose.pose.pose.position.x;
  pf_init_pose_mean.v[1] = last_published_pose.pose.pose.position.y;
  pf_init_pose_mean.v[2] = tf::getYaw(last_published_pose.pose.pose.orientation);
  pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
  pf_init_pose_cov.m[0][0] = last_published_pose.pose.covariance[6*0+0];
  pf_init_pose_cov.m[1][1] = last_published_pose.pose.covariance[6*1+1];
  pf_init_pose_cov.m[2][2] = last_published_pose.pose.covariance[6*5+5];
  pf_init(pf_, pf_init_pose_mean, pf_init_pose_cov);
  pf_init_ = false;

  // Instantiate the sensor objects
  // Odometry
  delete odom_;
  odom_ = new AMCLOdom();
  ROS_ASSERT(odom_);
  odom_->SetModel( odom_model_type_, alpha1_, alpha2_, alpha3_, alpha4_, alpha5_ );
  odom_frame_id_ = config.odom_frame_id;
  base_frame_id_ = config.base_frame_id;
  global_frame_id_ = config.global_frame_id;

  if(map_type_ == 2)
  {
    reconfigure2D(config);
  }
  else if(map_type_ == 3)
  {
    reconfigure3D(config);
  }

  save_pose_ = config.save_pose;
  const std::string filename = config.saved_pose_filename;
  saved_pose_filepath_ = makeFilepathFromName(filename);

  initial_pose_sub_ = nh_.subscribe("initialpose", 2, &AmclNode::initialPoseReceived, this);
}

void
AmclNode::loadPose()
{
  if(loadPoseFromServer())
  {
    ROS_DEBUG("Successfully loaded pose from server.");
  }
  else if(loadPoseFromFile())
  {
    ROS_INFO("Failed to load pose from server.");
    ROS_DEBUG("Successfully loaded pose from file.");
  }
  else
  {
    ROS_WARN("Failed to load pose from server or file.");
    ROS_INFO("Setting pose to default values.");
    init_pose_[0] = 0.0;
    init_pose_[1] = 0.0;
    init_pose_[2] = 0.0;
    init_cov_[0] = 0.5 * 0.5;
    init_cov_[1] = 0.5 * 0.5;
    init_cov_[2] = (M_PI/12.0) * (M_PI/12.0);
  }
  publishInitPose();
}

void
AmclNode::publishInitPose()
{
  geometry_msgs::PoseWithCovarianceStamped pose;
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = "/map";
  pose.pose.pose.position.x = init_pose_[0];
  pose.pose.pose.position.y = init_pose_[1];
  pose.pose.pose.position.z = 0.0;
  pose.pose.pose.orientation = tf::createQuaternionMsgFromYaw(init_pose_[2]);
  std::vector<double> cov_vals(36, 0.0);
  cov_vals[INDEX_XX_] = init_cov_[0];
  cov_vals[INDEX_YY_] = init_cov_[1];
  cov_vals[INDEX_AA_] = init_cov_[2];
  for(int i = 0; i < cov_vals.size(); i++)
  {  pose.pose.covariance[i] = cov_vals[i];  }
  initial_pose_pub_.publish(pose);
}

bool AmclNode::loadPoseFromServer()
{
  ROS_INFO("Loading pose from server");

  double tmp_pos;
  bool success;

  success = private_nh_.getParam("initial_pose_x", tmp_pos);
  if((not success) or std::isnan(tmp_pos))
  {
    ROS_DEBUG("Failed to load initial pose X from server.");
    return false;
  }
  else
  {
    init_pose_[0] = tmp_pos;
  }

  success = private_nh_.getParam("initial_pose_y", tmp_pos);
  if((not success) or std::isnan(tmp_pos))
  {
    ROS_DEBUG("Failed to load initial pose Y from server.");
    return false;
  }
  else
  {
    init_pose_[1] = tmp_pos;
  }

  success = private_nh_.getParam("initial_pose_a", tmp_pos);
  if((not success) or std::isnan(tmp_pos))
  {
    ROS_DEBUG("Failed to load initial pose Yaw from server.");
    return false;
  }
  else
  {
    init_pose_[2] = tmp_pos;
  }

  success = private_nh_.getParam("initial_cov_xx", tmp_pos);
  if((not success) or std::isnan(tmp_pos))
  {
    ROS_DEBUG("Failed to load initial covariance XX from server.");
    return false;
  }
  else
  {
    init_cov_[0] = tmp_pos;
  }

  success = private_nh_.getParam("initial_cov_yy", tmp_pos);
  if((not success) or std::isnan(tmp_pos))
  {
    ROS_DEBUG("Failed to load initial covariance YY from server.");
    return false;
  }
  else
  {
    init_cov_[1] = tmp_pos;
  }

  success = private_nh_.getParam("initial_cov_aa", tmp_pos);
  if((not success) or std::isnan(tmp_pos))
  {
    ROS_DEBUG("Failed to load initial covariance AA from server.");
    return false;
  }
  else
  {
    init_cov_[2] = tmp_pos;
  }
  ROS_INFO("Successfully loaded all initial pose parameters from server.");
  ROS_INFO("Pose loaded: %.3f, %.3f", init_pose_[0], init_pose_[1]);
  return true;
}

bool AmclNode::loadPoseFromFile()
{
  ROS_INFO("Loading pose from file");
  double x, y, z, w, roll, pitch, yaw, xx, yy, aa;
  try
  {
    YAML::Node config = loadYamlFromFile();
    x = config["pose"]["pose"]["position"]["x"].as<double>();
    y = config["pose"]["pose"]["position"]["y"].as<double>();
    z = config["pose"]["pose"]["orientation"]["z"].as<double>();
    w = config["pose"]["pose"]["orientation"]["w"].as<double>();
    tf::Quaternion q(0.0, 0.0, z, w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    xx = config["pose"]["covariance"][INDEX_XX_].as<double>();
    yy = config["pose"]["covariance"][INDEX_YY_].as<double>();
    aa = config["pose"]["covariance"][INDEX_AA_].as<double>();
  }
  catch(std::exception& e)
  {
    ROS_WARN("exception: %s", e.what());
    ROS_DEBUG("Failed to parse saved YAML pose.");
    return false;
  }
  if(std::isnan(x) or std::isnan(y) or std::isnan(yaw) or std::isnan(xx) or std::isnan(yy) or std::isnan(aa))
  {
      ROS_WARN("Failed to parse saved YAML pose. NAN value read from file.");
      return false;
  }
  init_pose_[0] = x;
  init_pose_[1] = y;
  init_pose_[2] = yaw;
  init_cov_[0] = xx;
  init_cov_[1] = yy;
  init_cov_[2] = aa;
  ROS_INFO("Successfully loaded YAML pose from file.");
  ROS_INFO("Pose loaded: %.3f, %.3f", init_pose_[0], init_pose_[1]);
  return true;
}

YAML::Node
AmclNode::loadYamlFromFile()
{
  YAML::Node node = YAML::LoadFile(saved_pose_filepath_);
  std::string key = node.begin()->first.as<std::string>();
  if(key.compare("header") == 0 or key.compare("pose") == 0)
  {
    ROS_DEBUG("YAML c++ style, returning node");
    return node;
  }
  else if(key.compare("state") == 0)
  {
    try
    {
      ROS_DEBUG("YAML python style, converting node");
      YAML::Node state_node = node["state"];
      YAML::Node header_node;
      header_node["frame_id"] = node["state"][0]["state"][2];
      YAML::Node position_node;
      position_node["x"] = node["state"][1]["state"][0]["state"][0]["state"][0];
      position_node["y"] = node["state"][1]["state"][0]["state"][0]["state"][1];
      YAML::Node orientation_node;
      orientation_node["z"] = node["state"][1]["state"][0]["state"][1]["state"][2];
      orientation_node["w"] = node["state"][1]["state"][0]["state"][1]["state"][3];
      YAML::Node pose_pose_node;
      pose_pose_node["position"] = position_node;
      pose_pose_node["orientation"] = orientation_node;
      YAML::Node pose_covariance_node;
      pose_covariance_node[INDEX_XX_] = node["state"][1]["state"][1][INDEX_XX_];
      pose_covariance_node[INDEX_YY_] = node["state"][1]["state"][1][INDEX_YY_];
      pose_covariance_node[INDEX_AA_] = node["state"][1]["state"][1][INDEX_AA_];
      YAML::Node pose_node;
      pose_node["pose"] = pose_pose_node;
      pose_node["covariance"] = pose_covariance_node;
      YAML::Node converted;
      converted["header"] = header_node;
      converted["pose"] = pose_node;
      return converted;
    }
    catch(std::exception& e)
    {
      YAML::Node empty;
      ROS_WARN("Exception thrown while parsing the saved pose file in the old Python style YAML.");
      return empty;
    }
  }
  else
  {
    YAML::Node empty;
    ROS_WARN("Cannot parse the saved pose file in either the new c++ style YAML nor the old Python style YAML.");
    return empty;
  }
}

void AmclNode::savePoseToServer()
{
  if(!save_pose_) {
    ROS_DEBUG("As specified, not saving pose to file");
    return;
  }
  // We need to apply the last transform to the latest odom pose to get
  // the latest map pose to store.  We'll take the covariance from
  // last_published_pose.
  tf::Pose map_pose = latest_tf_.inverse() * latest_odom_pose_;
  double yaw,pitch,roll;
  map_pose.getBasis().getEulerYPR(yaw, pitch, roll);

  ROS_DEBUG("Saving pose to server. x: %.3f, y: %.3f", map_pose.getOrigin().x(), map_pose.getOrigin().y() );

  private_nh_.setParam("initial_pose_x", map_pose.getOrigin().x());
  private_nh_.setParam("initial_pose_y", map_pose.getOrigin().y());
  private_nh_.setParam("initial_pose_a", yaw);
  private_nh_.setParam("initial_cov_xx",
                                  last_published_pose.pose.covariance[INDEX_XX_]);
  private_nh_.setParam("initial_cov_yy",
                                  last_published_pose.pose.covariance[INDEX_YY_]);
  private_nh_.setParam("initial_cov_aa",
                                  last_published_pose.pose.covariance[INDEX_AA_]);
  geometry_msgs::Pose pose;
  tf::poseTFToMsg(map_pose, pose);
  boost::recursive_mutex::scoped_lock lpl(latest_amcl_pose_mutex_);
  latest_amcl_pose_.pose.pose = pose;
  latest_amcl_pose_.pose.covariance[INDEX_XX_] = last_published_pose.pose.covariance[INDEX_XX_];
  latest_amcl_pose_.pose.covariance[INDEX_YY_] = last_published_pose.pose.covariance[INDEX_YY_];
  latest_amcl_pose_.pose.covariance[INDEX_AA_] = last_published_pose.pose.covariance[INDEX_AA_];
  latest_amcl_pose_.header.stamp = ros::Time::now();
  latest_amcl_pose_.header.frame_id = "map";
}

void AmclNode::savePoseToFile()
{
  if(!save_pose_) {
    return;
  }
  ROS_DEBUG("saving pose to file log warn");
  boost::recursive_mutex::scoped_lock lpl(latest_amcl_pose_mutex_);

  YAML::Node stamp_node;
  ros::Time stamp = latest_amcl_pose_.header.stamp;
  stamp_node["sec"] = stamp.sec;
  stamp_node["nsec"] = stamp.nsec;

  YAML::Node header_node;
  header_node["stamp"] = stamp_node;
  header_node["frame_id"] = "map";

  YAML::Node pose_pose_position_node;
  pose_pose_position_node["x"] = latest_amcl_pose_.pose.pose.position.x;
  pose_pose_position_node["y"] = latest_amcl_pose_.pose.pose.position.y;
  pose_pose_position_node["z"] = 0.0;

  YAML::Node pose_pose_orientation_node;
  pose_pose_orientation_node["x"] = 0.0;
  pose_pose_orientation_node["y"] = 0.0;
  pose_pose_orientation_node["z"] = latest_amcl_pose_.pose.pose.orientation.z;
  pose_pose_orientation_node["w"] = latest_amcl_pose_.pose.pose.orientation.w;

  YAML::Node pose_pose_node;
  pose_pose_node["position"] = pose_pose_position_node;
  pose_pose_node["orientation"] = pose_pose_orientation_node;

  YAML::Node pose_covariance_node;
  std::vector<double> covariance(36, 0.0);
  covariance[INDEX_XX_] = latest_amcl_pose_.pose.covariance[INDEX_XX_];
  covariance[INDEX_YY_] = latest_amcl_pose_.pose.covariance[INDEX_YY_];
  covariance[INDEX_AA_] = latest_amcl_pose_.pose.covariance[INDEX_AA_];
  for(int i = 0; i < covariance.size(); i++)
  {
    pose_covariance_node[i] = covariance[i];
  }

  YAML::Node pose_node;
  pose_node["pose"] = pose_pose_node;
  pose_node["covariance"] = pose_covariance_node;

  YAML::Node pose_stamped_node;
  pose_stamped_node["header"] = header_node;
  pose_stamped_node["pose"] = pose_node;

  badger_file_lib::atomic_ofstream file_buf(saved_pose_filepath_);
  file_buf << pose_stamped_node;
  file_buf.close();
}

std::string
AmclNode::makeFilepathFromName(const std::string filename)
{
  const char* bar_common = std::getenv("BAR_COMMON");
  if(bar_common == nullptr) {
    bar_common = "/var/snap/bar-base/common";
  }
  std::string bar_common_string = bar_common;
  return bar_common_string + "/" + filename;
}

void
AmclNode::initFromNewMap()
{

  // Create the particle filter
  pf_ = pf_alloc(min_particles_, max_particles_,
                 alpha_slow_, alpha_fast_,
                 (pf_init_model_fn_t)AmclNode::uniformPoseGenerator,
                 (void *)this);
  pf_->pop_err = pf_err_;
  pf_->pop_z = pf_z_;
  pf_set_resample_model(pf_, resample_model_type_);

  pf_vector_t pf_init_pose_mean = pf_vector_zero();
  pf_init_pose_mean.v[0] = init_pose_[0];
  pf_init_pose_mean.v[1] = init_pose_[1];
  pf_init_pose_mean.v[2] = init_pose_[2];
  pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
  pf_init_pose_cov.m[0][0] = init_cov_[0];
  pf_init_pose_cov.m[1][1] = init_cov_[1];
  pf_init_pose_cov.m[2][2] = init_cov_[2];
  pf_init(pf_, pf_init_pose_mean, pf_init_pose_cov);
  pf_init_ = false;

  // Instantiate the sensor objects
  // Odometry
  delete odom_;
  odom_ = new AMCLOdom();
  ROS_ASSERT(odom_);
  odom_->SetModel( odom_model_type_, alpha1_, alpha2_, alpha3_, alpha4_, alpha5_ );

  if(map_type_ == 2)
  {
    initFromNewMap2D();
  }
  else if(map_type_ == 3)
  {
    initFromNewMap3D();
  }

  // In case the initial pose message arrived before the first map,
  // try to apply the initial pose now that the map has arrived.
  applyInitialPose();
}

void
AmclNode::freeMapDependentMemory()
{
  if( map_ != NULL ) {
    delete map_;
    map_ = NULL;
  }
  if( pf_ != NULL ) {
    pf_free( pf_ );
    pf_ = NULL;
  }
  delete odom_;
  odom_ = NULL;
  if(map_type_ == 2)
  {
    freeMapDependentMemory2D();
  }
  else if(map_type_ == 3)
  {
    freeMapDependentMemory3D();
  }
}

AmclNode::~AmclNode()
{
  delete dsrv_;
  freeMapDependentMemory();
  deleteAmclNode2D();
  delete tfb_;
  delete tf_;
  // TODO: delete everything allocated in constructor
}

void
AmclNode::initOdomIntegrator()
{
  odom_integrator_ready_ = false;
}

void
AmclNode::resetOdomIntegrator()
{
  odom_integrator_absolute_motion_ = pf_vector_zero();
}

void
AmclNode::integrateOdom(const nav_msgs::OdometryConstPtr& msg)
{
  // Integrate absolute motion relative to the base,
  // by finding the delta from one odometry message to another.
  // NOTE: assume this odom topic is from our odom frame to our base frame.
  tf::Pose tf_pose;
  poseMsgToTF(msg->pose.pose, tf_pose);
  pf_vector_t pose;
  pose.v[0] = tf_pose.getOrigin().x();
  pose.v[1] = tf_pose.getOrigin().y();
  double yaw,pitch,roll;
  tf_pose.getBasis().getEulerYPR(yaw, pitch, roll);
  pose.v[2] = yaw;

  if (!odom_integrator_ready_) {
    resetOdomIntegrator();
    odom_integrator_ready_ = true;
  } else {
    pf_vector_t delta;

    delta.v[0] = pose.v[0] - odom_integrator_last_pose_.v[0];
    delta.v[1] = pose.v[1] - odom_integrator_last_pose_.v[1];
    delta.v[2] = angle_diff(pose.v[2], odom_integrator_last_pose_.v[2]);

    // project bearing change onto average orientation, x is forward translation, y is strafe
    double delta_trans, delta_rot, delta_bearing;
    delta_trans = sqrt(delta.v[0]*delta.v[0] + delta.v[1]*delta.v[1]);
    delta_rot = delta.v[2];
    if (delta_trans < 1e-6)
    {
      // For such a small translation, we either didn't move or rotated in place.
      // Assume the very small motion was forward, not strafe.
      delta_bearing = 0;
    }
    else
    {
      delta_bearing = angle_diff(atan2(delta.v[1], delta.v[0]),
                                 odom_integrator_last_pose_.v[2] + delta_rot/2);
    }
    double cs_bearing = cos(delta_bearing);
    double sn_bearing = sin(delta_bearing);

    // Accumulate absolute motion
    odom_integrator_absolute_motion_.v[0] += fabs(delta_trans * cs_bearing);
    odom_integrator_absolute_motion_.v[1] += fabs(delta_trans * sn_bearing);
    odom_integrator_absolute_motion_.v[2] += fabs(delta_rot);

    // We could also track velocity and acceleration here, for motion models that adjust for velocity/acceleration.
    // We could also track the covariance of the odometry message and accumulate a total covariance across the time
    // region for a motion model that uses the reported covariance directly.
  }
  odom_integrator_last_pose_ = pose;
}

bool
AmclNode::getOdomPose(tf::Stamped<tf::Pose>& odom_pose,
                      double& x, double& y, double& yaw,
                      const ros::Time& t, const std::string& f)
{
  // Get the robot's pose
  tf::Stamped<tf::Pose> ident (tf::Transform(tf::createIdentityQuaternion(),
                                           tf::Vector3(0,0,0)), t, f);
  try
  {
    this->tf_->transformPose(odom_frame_id_, ident, odom_pose);
  }
  catch(tf::TransformException e)
  {
    ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
    return false;
  }
  x = odom_pose.getOrigin().x();
  y = odom_pose.getOrigin().y();
  double pitch,roll;
  odom_pose.getBasis().getEulerYPR(yaw, pitch, roll);

  return true;
}

// Helper function to generate a random free-space pose
pf_vector_t
AmclNode::randomFreeSpacePose()
{
  Map* map = this->map_;
  pf_vector_t p;

  unsigned int rand_index = drand48() * free_space_indices.size();
  std::pair<int,int> free_point = free_space_indices[rand_index];
  std::vector<double> p_vec;
  p_vec = map_->convertMapToWorld({free_point.first, free_point.second});
  p.v[0] = p_vec[0];
  p.v[1] = p_vec[1];
  p.v[2] = drand48() * 2 * M_PI - M_PI;

  return p;
}

// Helper function to score a pose for uniform pose generation
double
AmclNode::scorePose(const pf_vector_t &p)
{
  if(map_type_ == 2)
  {
    return scorePose2D(p);
  }
  else if(map_type_ == 3)
  {
    return scorePose3D(p);
  }
  else
  {
    ROS_ERROR("invalid map type");
    return -1;
  }
}

pf_vector_t
AmclNode::uniformPoseGenerator(void* arg)
{
  AmclNode *self = (AmclNode*)arg;
  double good_weight = self->uniform_pose_starting_weight_threshold_;
  const double deweight_multiplier = self->uniform_pose_deweight_multiplier_;
  pf_vector_t p;

  p = self->randomFreeSpacePose();

  if(self->map_type_ == 2 and self->last_laser_data_ == NULL)
      return p;

  // Check and see how "good" this pose is.
  // Begin with the configured starting weight threshold,
  // then down-weight each try by the configured deweight multiplier.
  // A starting weight of 0 or negative means disable this check.
  // Also sanitize the value of deweight_multiplier.
  if (good_weight > 0.0 && deweight_multiplier < 1.0 && deweight_multiplier >= 0.0)
  {
    while (self->scorePose(p) < good_weight)
    {
      p = self->randomFreeSpacePose();
      good_weight *= deweight_multiplier;
    }
  }

  return p;
}

bool
AmclNode::globalLocalizationCallback(std_srvs::Empty::Request& req,
                                     std_srvs::Empty::Response& res)
{
  if( map_ == NULL ) {
    return true;
  }
  boost::recursive_mutex::scoped_lock cfl(configuration_mutex_);
  global_localization_active_ = true;
  pf_->alpha_slow = global_localization_alpha_slow_;
  pf_->alpha_fast = global_localization_alpha_fast_;
  if(map_type_ == 2)
  {
    globalLocalizationCallback2D();
  }
  else if(map_type_ == 3)
  {
    globalLocalizationCallback3D();
  }

  ROS_INFO("Initializing with uniform distribution");
  pf_init_model(pf_, (pf_init_model_fn_t)AmclNode::uniformPoseGenerator,
                (void *)this);
  ROS_INFO("Global initialisation done!");
  pf_init_ = false;
  return true;
}

void
AmclNode::publishTransform(const ros::TimerEvent& event)
{
  boost::recursive_mutex::scoped_lock tfl(tf_mutex_);
  if (tf_broadcast_ == true && latest_tf_valid_)
  {
    // We want to send a transform that is good up until a
    // tolerance time so that odom can be used
    ros::Time transform_expiration = (ros::Time::now() + transform_tolerance_);
    tf::StampedTransform tmp_tf_stamped;
    tf::Transform tf_transform;
    if (tf_reverse_ == true)
    {
      tmp_tf_stamped = tf::StampedTransform(latest_tf_, transform_expiration,
                                            odom_frame_id_, global_frame_id_);
      tf_transform = latest_tf_;
    } else {
      tmp_tf_stamped = tf::StampedTransform(latest_tf_.inverse(), transform_expiration,
                                            global_frame_id_, odom_frame_id_);
      tf_transform = latest_tf_.inverse();
    }
    geometry_msgs::Quaternion quaternion;
    tf::quaternionTFToMsg(tf_transform.getRotation(), quaternion);
    geometry_msgs::Vector3 origin;
    tf::vector3TFToMsg(tf_transform.getOrigin(), origin);
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = global_frame_id_;
    odom.child_frame_id = odom_frame_id_;
    odom.pose.pose.position.x = origin.x;
    odom.pose.pose.position.y = origin.y;
    odom.pose.pose.position.z = origin.z;
    odom.pose.pose.orientation = quaternion;
    map_odom_transform_pub_.publish(odom);

    this->tfb_->sendTransform(tmp_tf_stamped);
    sent_first_transform_ = true;
  }
}

double
AmclNode::getYaw(tf::Pose& t)
{
  double yaw, pitch, roll;
  t.getBasis().getEulerYPR(yaw,pitch,roll);
  return yaw;
}

void
AmclNode::initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  handleInitialPoseMessage(*msg);
}

void
AmclNode::handleInitialPoseMessage(const geometry_msgs::PoseWithCovarianceStamped& orig_msg)
{
  ROS_INFO("initial pose received");
  boost::recursive_mutex::scoped_lock cfl(configuration_mutex_);
  geometry_msgs::PoseWithCovarianceStamped msg(orig_msg);
  // Rewrite to our global frame if received in the alt frame.
  // This allows us to run with multiple localizers using tf_reverse and pose them all at once.
  // And it is much cheaper to rewrite here than to run a separate topic tool transformer.
  if(tf_->resolve(msg.header.frame_id) == tf_->resolve(global_alt_frame_id_))
  {
    msg.header.frame_id = global_frame_id_;
  }
  if(msg.header.frame_id == "")
  {
    // This should be removed at some point
    ROS_WARN("Received initial pose with empty frame_id.  You should always supply a frame_id.");
  }
  // We only accept initial pose estimates in the global frame, #5148.
  else if(tf_->resolve(msg.header.frame_id) != tf_->resolve(global_frame_id_))
  {
    ROS_WARN("Ignoring initial pose in frame \"%s\"; initial poses must be in the global frame, \"%s\"",
             msg.header.frame_id.c_str(),
             global_frame_id_.c_str());
    return;
  }

  if(std::isnan(msg.pose.pose.position.x)
     or std::isnan(msg.pose.pose.position.y)
     or std::isnan(msg.pose.pose.position.z))
  {
    ROS_WARN("Received initial pose with position value 'NAN'. Ignoring pose.");
    return;
  }

  if(std::isnan(msg.pose.pose.orientation.x)
     or std::isnan(msg.pose.pose.orientation.y)
     or std::isnan(msg.pose.pose.orientation.z)
     or std::isnan(msg.pose.pose.orientation.w))
  {
    ROS_WARN("Received initial pose with orientation value 'NAN'. Ignoring pose.");
    return;
  }

  std::vector<double> default_cov_vals(36, 0.0);
  default_cov_vals[INDEX_XX_] = 0.5 * 0.5;
  default_cov_vals[INDEX_YY_] = 0.5 * 0.5;
  default_cov_vals[INDEX_AA_] = (M_PI/12.0) * (M_PI/12.0);
  for(int i = 0; i < msg.pose.covariance.size(); i++)
  {
    if(std::isnan(msg.pose.covariance[i]))
    {
      msg.pose.covariance[i] = default_cov_vals[i];
    }
  }

  // In case the client sent us a pose estimate in the past, integrate the
  // intervening odometric change.
  tf::StampedTransform tx_odom;
  try
  {
    ros::Time now = ros::Time::now();
    // wait a little for the latest tf to become available
    tf_->waitForTransform(base_frame_id_, msg.header.stamp,
                         base_frame_id_, now,
                         odom_frame_id_, ros::Duration(0.5));
    tf_->lookupTransform(base_frame_id_, msg.header.stamp,
                         base_frame_id_, now,
                         odom_frame_id_, tx_odom);
  }
  catch(tf::TransformException e)
  {
    // If we've never sent a transform, then this is normal, because the
    // global_frame_id_ frame doesn't exist.  We only care about in-time
    // transformation for on-the-move pose-setting, so ignoring this
    // startup condition doesn't really cost us anything.
    boost::recursive_mutex::scoped_lock tfl(tf_mutex_);
    if(sent_first_transform_)
      ROS_WARN("Failed to transform initial pose in time (%s)", e.what());
    tx_odom.setIdentity();
  }

  tf::Pose pose_old, pose_new;
  tf::poseMsgToTF(msg.pose.pose, pose_old);
  pose_new = pose_old * tx_odom;

  // Transform into the global frame

  ROS_INFO("Setting pose (%.6f): %.3f %.3f %.3f",
           ros::Time::now().toSec(),
           pose_new.getOrigin().x(),
           pose_new.getOrigin().y(),
           getYaw(pose_new));
  // Re-initialize the filter
  pf_vector_t pf_init_pose_mean = pf_vector_zero();
  pf_init_pose_mean.v[0] = pose_new.getOrigin().x();
  pf_init_pose_mean.v[1] = pose_new.getOrigin().y();
  pf_init_pose_mean.v[2] = getYaw(pose_new);
  pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
  // Copy in the covariance, converting from 6-D to 3-D
  for(int i=0; i<2; i++)
  {
    for(int j=0; j<2; j++)
    {
      pf_init_pose_cov.m[i][j] = msg.pose.covariance[6*i+j];
    }
  }
  pf_init_pose_cov.m[2][2] = msg.pose.covariance[6*5+5];

  delete initial_pose_hyp_;
  initial_pose_hyp_ = new amcl_hyp_t();
  initial_pose_hyp_->pf_pose_mean = pf_init_pose_mean;
  initial_pose_hyp_->pf_pose_cov = pf_init_pose_cov;
  applyInitialPose();

  // disable global localization in case it was active
  global_localization_active_ = false;
}

/**
 * If initial_pose_hyp_ and map_ are both non-null, apply the initial
 * pose to the particle filter state.  initial_pose_hyp_ is deleted
 * and set to NULL after it is used.
 */
void
AmclNode::applyInitialPose()
{
  boost::recursive_mutex::scoped_lock cfl(configuration_mutex_);
  if( initial_pose_hyp_ != NULL && map_ != NULL ) {
    pf_init(pf_, initial_pose_hyp_->pf_pose_mean, initial_pose_hyp_->pf_pose_cov);
    pf_init_ = false;

    delete initial_pose_hyp_;
    initial_pose_hyp_ = NULL;
  }
}

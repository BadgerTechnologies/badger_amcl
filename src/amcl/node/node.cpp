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

#include "node/node.h"

#include <stdlib.h>

#include <cstdlib>
#include <functional>

#include <angles/angles.h>
#include <badger_file_lib/atomic_ofstream.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <ros/assert.h>
#include <ros/console.h>
#include <tf2/exceptions.h>
#include <tf2/utils.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "node/node_2d.h"
#include "node/node_3d.h"

namespace badger_amcl
{

Node::Node()
  : sent_first_transform_(false),
    latest_tf_valid_(false),
    map_(NULL),
    private_nh_("~"),
    initial_pose_hyp_(NULL),
    first_reconfigure_call_(true),
    publish_transform_spinner_(1, &publish_transform_queue_),
    global_localization_active_(false),
    dsrv_(ros::NodeHandle("~")),
    tf_listener_(tf_buffer_)
{
  std::lock_guard<std::mutex> cfl(configuration_mutex_);

  private_nh_.param("map_type", map_type_, 0);

  double param_val;
  private_nh_.param("transform_publish_rate", param_val, 50.0);
  transform_publish_period_ = ros::Duration(1.0 / param_val);
  private_nh_.param("save_pose_to_file_rate", param_val, 0.1);
  save_pose_to_file_period_ = ros::Duration(1.0 / param_val);

  private_nh_.param("min_particles", min_particles_, 100);
  private_nh_.param("max_particles", max_particles_, 5000);
  private_nh_.param("kld_err", pf_err_, 0.01);
  private_nh_.param("kld_z", pf_z_, 0.99);
  private_nh_.param("odom_integrator_enabled", odom_integrator_enabled_, true);
  private_nh_.param("odom_alpha1", alpha1_, 0.2);
  private_nh_.param("odom_alpha2", alpha2_, 0.2);
  private_nh_.param("odom_alpha3", alpha3_, 0.2);
  private_nh_.param("odom_alpha4", alpha4_, 0.2);
  private_nh_.param("odom_alpha5", alpha5_, 0.2);
  private_nh_.param("global_localization_convergence_threshold", global_localization_convergence_threshold_, 95.0);

  private_nh_.param("save_pose", save_pose_, false);
  const std::string default_filepath = "badger_amcl_saved_pose.yaml";
  private_nh_.param("saved_pose_filepath", saved_pose_filepath_, default_filepath);

  std::string model_type_str;
  private_nh_.param("odom_model_type", model_type_str, std::string("diff"));
  if (model_type_str == "diff")
    odom_model_type_ = ODOM_MODEL_DIFF;
  else if (model_type_str == "omni")
    odom_model_type_ = ODOM_MODEL_OMNI;
  else if (model_type_str == "diff-corrected")
    odom_model_type_ = ODOM_MODEL_DIFF_CORRECTED;
  else if (model_type_str == "omni-corrected")
    odom_model_type_ = ODOM_MODEL_OMNI_CORRECTED;
  else if (model_type_str == "gaussian")
    odom_model_type_ = ODOM_MODEL_GAUSSIAN;
  else
  {
    ROS_WARN_STREAM("Unknown odom model type \"" << model_type_str << "\"; defaulting to diff model");
    odom_model_type_ = ODOM_MODEL_DIFF;
  }

  private_nh_.param("update_min_d", d_thresh_, 0.2);
  private_nh_.param("update_min_a", a_thresh_, M_PI / 6.0);
  private_nh_.param("odom_frame_id", odom_frame_id_, std::string("odom"));
  private_nh_.param("base_frame_id", base_frame_id_, std::string("base_link"));
  private_nh_.param("global_frame_id", global_frame_id_, std::string("map"));
  private_nh_.param("global_alt_frame_id", global_alt_frame_id_, std::string(""));
  private_nh_.param("resample_model_type", model_type_str, std::string("multinomial"));
  if (model_type_str == "multinomial")
    resample_model_type_ = PF_RESAMPLE_MULTINOMIAL;
  else if (model_type_str == "systematic")
    resample_model_type_ = PF_RESAMPLE_SYSTEMATIC;
  else
  {
    ROS_WARN_STREAM("Unknown resample model type \"" << model_type_str << "\"; defaulting to multinomial model");
    resample_model_type_ = PF_RESAMPLE_MULTINOMIAL;
  }

  double transform_tolerance_val;
  private_nh_.param("transform_tolerance", transform_tolerance_val, 0.1);
  private_nh_.param("recovery_alpha_slow", alpha_slow_, 0.001);
  private_nh_.param("recovery_alpha_fast", alpha_fast_, 0.1);
  private_nh_.param("uniform_pose_starting_weight_threshold", uniform_pose_starting_weight_threshold_, 0.0);
  private_nh_.param("uniform_pose_deweight_multiplier", uniform_pose_deweight_multiplier_, 0.0);
  private_nh_.param("global_localization_alpha_slow", global_localization_alpha_slow_, 0.001);
  private_nh_.param("global_localization_alpha_fast", global_localization_alpha_fast_, 0.1);
  private_nh_.param("tf_broadcast", tf_broadcast_, true);
  private_nh_.param("tf_reverse", tf_reverse_, false);

  transform_tolerance_.fromSec(transform_tolerance_val);

  initial_pose_sub_ = nh_.subscribe("initialpose", 2, &Node::initialPoseReceived, this);

  pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 2, true);
  particlecloud_pub_ = nh_.advertise<geometry_msgs::PoseArray>("particlecloud", 2, true);
  if (global_alt_frame_id_.size() > 0)
  {
    alt_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose_in_" + global_alt_frame_id_,
                                                                            2, true);
    alt_particlecloud_pub_ = nh_.advertise<geometry_msgs::PoseArray>("particlecloud_in_" + global_alt_frame_id_,
                                                                     2, true);
  }
  map_odom_transform_pub_ = nh_.advertise<nav_msgs::Odometry>("amcl_map_odom_transform", 1);
  global_loc_srv_ = nh_.advertiseService("global_localization", &Node::globalLocalizationCallback, this);

  default_cov_vals_.resize(36, 0.0);
  default_cov_vals_[COVARIANCE_XX] = 0.5 * 0.5;
  default_cov_vals_[COVARIANCE_YY] = 0.5 * 0.5;
  default_cov_vals_[COVARIANCE_AA] = (M_PI / 12.0) * (M_PI / 12.0);
  loadPose();

  if(odom_integrator_enabled_);
  {
    std::string odom_integrator_topic = "odom";
    odom_integrator_sub_ = nh_.subscribe(odom_integrator_topic, 20, &Node::integrateOdom, this);
    absolute_motion_pub_ = nh_.advertise<geometry_msgs::Pose2D>("amcl_absolute_motion", 20, false);
  }

  if(map_type_ == 2)
  {
    node_ = std::make_shared<Node2D>(this, configuration_mutex_);
  }
  if(map_type_ == 3)
  {
    node_ = std::make_shared<Node3D>(this, configuration_mutex_);
  }

  dynamic_reconfigure::Server<AMCLConfig>::CallbackType cb = std::bind(&Node::reconfigureCB, this,
                                                                       std::placeholders::_1, std::placeholders::_2);
  dsrv_.setCallback(cb);

  publish_transform_nh_ = nh_;
  publish_transform_nh_.setCallbackQueue(&publish_transform_queue_);
  publish_transform_timer_ = publish_transform_nh_.createTimer(transform_publish_period_,
                                                               std::bind(&Node::publishTransform, this,
                                                                         std::placeholders::_1));
  publish_transform_spinner_.start();

  // Save the pose on its own timer so if we get blocked for a long time on IO
  // we can still localize as sensor data comes in. This works fine because we
  // run on a multi-threaded spinner by design.
  save_pose_to_file_timer_ = nh_.createTimer(
      save_pose_to_file_period_,
      std::bind(&Node::attemptSavePose, this, false));
}

void Node::reconfigureCB(AMCLConfig& config, uint32_t level)
{
  // we don't want to do anything on the first call
  // which corresponds to startup
  if (first_reconfigure_call_)
  {
    first_reconfigure_call_ = false;
    default_config_ = config;
    return;
  }

  std::lock_guard<std::mutex> cfl(configuration_mutex_);

  if (config.restore_defaults)
  {
    config = default_config_;
    // avoid looping
    config.restore_defaults = false;
  }

  d_thresh_ = config.update_min_d;
  a_thresh_ = config.update_min_a;

  if (config.resample_model_type == "multinomial")
    resample_model_type_ = PF_RESAMPLE_MULTINOMIAL;
  else if (config.resample_model_type == "systematic")
    resample_model_type_ = PF_RESAMPLE_SYSTEMATIC;
  else
  {
    ROS_WARN_STREAM("Unknown resample model type \"" << config.resample_model_type
                    << "\"; defaulting to multinomial model");
    resample_model_type_ = PF_RESAMPLE_MULTINOMIAL;
  }

  transform_publish_period_ = ros::Duration(1.0 / config.transform_publish_rate);
  save_pose_to_file_period_ = ros::Duration(1.0 / config.save_pose_to_file_rate);

  transform_tolerance_.fromSec(config.transform_tolerance);

  alpha1_ = config.odom_alpha1;
  alpha2_ = config.odom_alpha2;
  alpha3_ = config.odom_alpha3;
  alpha4_ = config.odom_alpha4;
  alpha5_ = config.odom_alpha5;

  if (config.odom_model_type == "diff")
    odom_model_type_ = ODOM_MODEL_DIFF;
  else if (config.odom_model_type == "omni")
    odom_model_type_ = ODOM_MODEL_OMNI;
  else if (config.odom_model_type == "diff-corrected")
    odom_model_type_ = ODOM_MODEL_DIFF_CORRECTED;
  else if (config.odom_model_type == "omni-corrected")
    odom_model_type_ = ODOM_MODEL_OMNI_CORRECTED;
  else if (config.odom_model_type == "gaussian")
    odom_model_type_ = ODOM_MODEL_GAUSSIAN;

  if (config.min_particles > config.max_particles)
  {
    ROS_WARN("You've set min_particles to be greater than max particles, "
             "this isn't allowed so they'll be set to be equal.");
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

  uniform_pose_generator_fn_ = std::bind(&Node::uniformPoseGenerator, this);
  pf_ = std::make_shared<ParticleFilter>(min_particles_, max_particles_, alpha_slow_, alpha_fast_,
                                         global_localization_convergence_threshold_, uniform_pose_generator_fn_);
  pf_err_ = config.kld_err;
  pf_z_ = config.kld_z;
  pf_->setPopulationSizeParameters(pf_err_, pf_z_);
  pf_->setResampleModel(resample_model_type_);

  // Initialize the filter
  Eigen::Vector3d pf_init_pose_mean;
  pf_init_pose_mean[0] = last_published_pose_->pose.pose.position.x;
  pf_init_pose_mean[1] = last_published_pose_->pose.pose.position.y;
  pf_init_pose_mean[2] = tf2::getYaw(last_published_pose_->pose.pose.orientation);
  Eigen::Matrix3d pf_init_pose_cov;
  pf_init_pose_cov(0, 0) = last_published_pose_->pose.covariance[COVARIANCE_XX];
  pf_init_pose_cov(1, 1) = last_published_pose_->pose.covariance[COVARIANCE_YY];
  pf_init_pose_cov(2, 2) = last_published_pose_->pose.covariance[COVARIANCE_AA];
  pf_->initWithGaussian(pf_init_pose_mean, pf_init_pose_cov);
  odom_init_ = false;

  // Instantiate the sensor objects
  // Odometry
  odom_.setModel(odom_model_type_, alpha1_, alpha2_, alpha3_, alpha4_, alpha5_);
  odom_frame_id_ = config.odom_frame_id;
  base_frame_id_ = config.base_frame_id;
  global_frame_id_ = config.global_frame_id;
  node_->reconfigure(config);
  save_pose_ = config.save_pose;
  saved_pose_filepath_ = config.saved_pose_filepath;
  publish_transform_timer_.setPeriod(transform_publish_period_);
  save_pose_to_file_timer_.setPeriod(save_pose_to_file_period_);
}

void Node::setPfDecayRateNormal()
{
  pf_->setDecayRates(alpha_slow_, alpha_fast_);
}

bool Node::updatePf(const ros::Time& t, std::vector<bool>& scanners_update, int scanner_index,
                    int* resample_count, bool* force_publication, bool* force_update)
{
  // Where the robot was when this scan was taken
  Eigen::Vector3d pose;
  if (getOdomPose(t, &pose))
  {
    Eigen::Vector3d delta;
    if(odom_init_)
    {
      computeDelta(pose, &delta);
      setScannersUpdateFlags(delta, scanners_update, force_update);
      if(scanners_update.at(scanner_index))
      {
        updateOdom(pose, delta);
      }
    }
    else
    {
      initOdom(pose, scanners_update, resample_count, force_publication);
    }
  }
  else
  {
    ROS_ERROR("Couldn't determine robot's pose associated with scan");
    return false;
  }
  return true;
}

std::shared_ptr<ParticleFilter> Node::getPfPtr()
{
  return pf_;
}

void Node::publishParticleCloud()
{
  std::shared_ptr<PFSampleSet> set = pf_->getCurrentSet();
  ROS_DEBUG_STREAM("Num samples: " << set->sample_count);
  geometry_msgs::PoseArray cloud_msg;
  cloud_msg.header.stamp = ros::Time::now();
  cloud_msg.header.frame_id = global_frame_id_;
  cloud_msg.poses.resize(set->sample_count);
  tf2::Quaternion q;
  for (int i = 0; i < set->sample_count; i++)
  {
    q.setRPY(0.0, 0.0, set->samples[i].pose[2]);
    tf2::toMsg(tf2::Transform(q, tf2::Vector3(set->samples[i].pose[0], set->samples[i].pose[1], 0)),
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

void Node::updatePose(const Eigen::Vector3d& max_hyp_mean, const ros::Time& stamp)
{
  std::shared_ptr<geometry_msgs::PoseWithCovarianceStamped> p = (
          std::make_shared<geometry_msgs::PoseWithCovarianceStamped>());
  // Fill in the header
  p->header.frame_id = global_frame_id_;
  p->header.stamp = stamp;
  // Copy in the pose
  p->pose.pose.position.x = max_hyp_mean[0];
  p->pose.pose.position.y = max_hyp_mean[1];
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, max_hyp_mean[2]);
  p->pose.pose.orientation = tf2::toMsg(q);
  // Copy in the covariance, converting from 3-D to 6-D
  std::shared_ptr<PFSampleSet> set = pf_->getCurrentSet();
  for (int i = 0; i < 2; i++)
  {
    for (int j = 0; j < 2; j++)
    {
      // Report the overall filter covariance, rather than the
      // covariance for the highest-weight cluster
      p->pose.covariance[6 * i + j] = set->cov(i, j);
    }
  }
  // Report the overall filter covariance, rather than the
  // covariance for the highest-weight cluster
  p->pose.covariance[COVARIANCE_AA] = set->cov(2, 2);
  publishPose(*p);
  std::lock_guard<std::mutex> lpl(latest_pose_mutex_);
  last_published_pose_ = p;
}

void Node::publishPose(const geometry_msgs::PoseWithCovarianceStamped& p)
{
  pose_pub_.publish(p);
  if (global_alt_frame_id_.size() > 0)
  {
    geometry_msgs::PoseWithCovarianceStamped alt_p(p);
    alt_p.header.frame_id = global_alt_frame_id_;
    alt_pose_pub_.publish(alt_p);
  }
}

void Node::updateOdomToMapTransform(const tf2::Transform& odom_to_map)
{
  std::lock_guard<std::mutex> tfl(tf_mutex_);
  latest_tf_ = odom_to_map;
  latest_tf_valid_ = true;
}

void Node::attemptSavePose(bool exiting)
{
  // When called on exit the multi-threaded spinner has already stopped, so
  // there is no need to worry about the direct call from main during exit
  // running concurrently with the timer callback.
  tf2::Transform latest_tf;
  if (getLatestTf(&latest_tf))
  {
    geometry_msgs::PoseWithCovarianceStamped latest_pose;
    getLatestPose(latest_tf, &latest_pose);
    savePoseToFile(latest_pose, exiting);
  }
}

void Node::loadPose()
{
  if (loadPoseFromFile())
  {
    ROS_INFO("Successfully loaded pose from file.");
    ROS_INFO("Pose loaded: (%.3f, %.3f)", init_pose_[0], init_pose_[1]);
  }
  else
  {
    ROS_WARN("Failed to load pose from file. Setting pose to default values.");
    init_pose_[0] = 0.0;
    init_pose_[1] = 0.0;
    init_pose_[2] = 0.0;
    init_cov_[0] = default_cov_vals_[COVARIANCE_XX];
    init_cov_[1] = default_cov_vals_[COVARIANCE_YY];
    init_cov_[2] = default_cov_vals_[COVARIANCE_AA];
    ROS_INFO("Default pose: (%.3f, %.3f)", init_pose_[0], init_pose_[1]);
  }
}

void Node::createInitialPose(tf2::Transform* pose, std::vector<double>* cov_vals)
{
  tf2::Vector3 origin(init_pose_[0], init_pose_[1], 0.0);
  tf2::Quaternion rotation;
  rotation.setRPY(0.0, 0.0, init_pose_[2]);
  pose->setOrigin(origin);
  pose->setRotation(rotation);
  cov_vals->at(COVARIANCE_XX) = init_cov_[0];
  cov_vals->at(COVARIANCE_YY) = init_cov_[1];
  cov_vals->at(COVARIANCE_AA) = init_cov_[2];
  ROS_INFO("Initial pose: (%.3f, %.3f)", pose->getOrigin().getX(), pose->getOrigin().getY());
}

bool Node::loadPoseFromFile()
{
  bool on_exit;
  double pose_x, pose_y, orientation_x, orientation_y, orientation_z, orientation_w, roll, pitch, yaw, xx, yy, aa;
  try
  {
    YAML::Node config = loadYamlFromFile();
    pose_x = config["pose"]["pose"]["position"]["x"].as<double>();
    pose_y = config["pose"]["pose"]["position"]["y"].as<double>();
    orientation_x = config["pose"]["pose"]["orientation"]["x"].as<double>();
    orientation_y = config["pose"]["pose"]["orientation"]["y"].as<double>();
    orientation_z = config["pose"]["pose"]["orientation"]["z"].as<double>();
    orientation_w = config["pose"]["pose"]["orientation"]["w"].as<double>();
    tf2::Quaternion q(orientation_x, orientation_y, orientation_z, orientation_w);
    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    xx = config["pose"]["covariance"][COVARIANCE_XX].as<double>();
    yy = config["pose"]["covariance"][COVARIANCE_YY].as<double>();
    aa = config["pose"]["covariance"][COVARIANCE_AA].as<double>();
    if(config["header"]["on_exit"])
      on_exit = config["header"]["on_exit"].as<bool>();
    else
      // assume pose was saved on exit if flag does not exist
      on_exit = true;
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM("Exception while loading pose from file. Failed to parse saved pose. " << e.what());
    return false;
  }
  if (std::isnan(pose_x) or std::isnan(pose_y)
      or std::isnan(orientation_x) or std::isnan(orientation_y)
      or std::isnan(orientation_z) or std::isnan(orientation_w)
      or std::isnan(xx) or std::isnan(yy) or std::isnan(aa))
  {
    ROS_WARN("Failed to parse saved YAML pose. NAN value read from file.");
    return false;
  }
  else if (std::isnan(yaw))
  {
    ROS_WARN("Failed to parse saved YAML pose. Rotation quaternion (%f, %f, %f, %f) invalid.",
             orientation_x, orientation_y, orientation_z, orientation_w);
    return false;
  }
  init_pose_[0] = pose_x;
  init_pose_[1] = pose_y;
  init_pose_[2] = yaw;
  if(on_exit)
  {
    init_cov_[0] = xx;
    init_cov_[1] = yy;
    init_cov_[2] = aa;
  }
  else
  {
    init_cov_[0] = default_cov_vals_[COVARIANCE_XX];
    init_cov_[1] = default_cov_vals_[COVARIANCE_YY];
    init_cov_[2] = default_cov_vals_[COVARIANCE_AA];
  }
  return true;
}

YAML::Node Node::loadYamlFromFile()
{
  YAML::Node node = YAML::LoadFile(saved_pose_filepath_);
  std::string key = node.begin()->first.as<std::string>();
  if (key.compare("header") == 0 or key.compare("pose") == 0)
  {
    ROS_DEBUG("YAML c++ style, returning node");
    return node;
  }
  else if (key.compare("state") == 0)
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
      pose_covariance_node[COVARIANCE_XX] = node["state"][1]["state"][1][COVARIANCE_XX];
      pose_covariance_node[COVARIANCE_YY] = node["state"][1]["state"][1][COVARIANCE_YY];
      pose_covariance_node[COVARIANCE_AA] = node["state"][1]["state"][1][COVARIANCE_AA];
      YAML::Node pose_node;
      pose_node["pose"] = pose_pose_node;
      pose_node["covariance"] = pose_covariance_node;
      YAML::Node converted;
      converted["header"] = header_node;
      converted["pose"] = pose_node;
      return converted;
    }
    catch (std::exception& e)
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

void Node::savePoseToFile(const geometry_msgs::PoseWithCovarianceStamped& latest_pose, bool save_on_exit)
{
  if (!save_pose_)
  {
    ROS_DEBUG("As specified, not saving pose to file.");
    return;
  }
  if (!latest_tf_valid_)
  {
    // We can enter this state on shutdown when the main function in main.cpp calls this function directly.
    ROS_DEBUG("TF is not valid, not saving pose to file.");
    return;
  }

  YAML::Node stamp_node;
  ros::Time stamp = latest_pose.header.stamp;
  stamp_node["sec"] = stamp.sec;
  stamp_node["nsec"] = stamp.nsec;

  YAML::Node header_node;
  header_node["stamp"] = stamp_node;
  header_node["frame_id"] = "map";
  header_node["on_exit"] = save_on_exit;

  YAML::Node pose_pose_position_node;
  pose_pose_position_node["x"] = latest_pose.pose.pose.position.x;
  pose_pose_position_node["y"] = latest_pose.pose.pose.position.y;
  pose_pose_position_node["z"] = 0.0;

  YAML::Node pose_pose_orientation_node;
  pose_pose_orientation_node["x"] = 0.0;
  pose_pose_orientation_node["y"] = 0.0;
  pose_pose_orientation_node["z"] = latest_pose.pose.pose.orientation.z;
  pose_pose_orientation_node["w"] = latest_pose.pose.pose.orientation.w;

  YAML::Node pose_pose_node;
  pose_pose_node["position"] = pose_pose_position_node;
  pose_pose_node["orientation"] = pose_pose_orientation_node;

  YAML::Node pose_covariance_node;
  std::vector<double> covariance(36, 0.0);
  covariance[COVARIANCE_XX] = latest_pose.pose.covariance[COVARIANCE_XX];
  covariance[COVARIANCE_YY] = latest_pose.pose.covariance[COVARIANCE_YY];
  covariance[COVARIANCE_AA] = latest_pose.pose.covariance[COVARIANCE_AA];
  for (int i = 0; i < covariance.size(); i++)
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

void Node::initFromNewMap(std::shared_ptr<Map> new_map, bool use_initial_pose)
{
  map_ = new_map;
  if(not use_initial_pose)
    return;

  // Create the particle filter
  uniform_pose_generator_fn_ = std::bind(&Node::uniformPoseGenerator, this);
  pf_ = std::make_shared<ParticleFilter>(min_particles_, max_particles_, alpha_slow_, alpha_fast_,
                                         global_localization_convergence_threshold_,uniform_pose_generator_fn_);
  pf_->setPopulationSizeParameters(pf_err_, pf_z_);
  pf_->setResampleModel(resample_model_type_);

  Eigen::Vector3d pf_init_pose_mean;
  pf_init_pose_mean[0] = init_pose_[0];
  pf_init_pose_mean[1] = init_pose_[1];
  pf_init_pose_mean[2] = init_pose_[2];
  Eigen::Matrix3d pf_init_pose_cov;
  for(int i = 0; i < 3; i++)
  {
    for(int j = 0; j < 3; j++)
    {
      pf_init_pose_cov(i, j) = 0.0;
    }
  }
  pf_init_pose_cov(0, 0) = init_cov_[0];
  pf_init_pose_cov(1, 1) = init_cov_[1];
  pf_init_pose_cov(2, 2) = init_cov_[2];
  pf_->initWithGaussian(pf_init_pose_mean, pf_init_pose_cov);
  odom_init_ = false;

  // Instantiate the sensor objects
  // Odometry
  odom_.setModel(odom_model_type_, alpha1_, alpha2_, alpha3_, alpha4_, alpha5_);

  tf2::Transform pose;
  std::vector<double> cov_vals(36, 0.0);
  createInitialPose(&pose, &cov_vals);
  setInitialPose(pose, cov_vals);
}

void Node::updateFreeSpaceIndices(std::vector<std::pair<int, int>> fsi)
{
  free_space_indices_ = fsi;
}

void Node::initOdomIntegrator()
{
  odom_integrator_ready_ = false;
}

void Node::resetOdomIntegrator()
{
  odom_integrator_absolute_motion_ = Eigen::Vector3d();
}

void Node::integrateOdom(const nav_msgs::OdometryConstPtr& msg)
{
  // Integrate absolute motion relative to the base,
  // by finding the delta from one odometry message to another.
  // NOTE: assume this odom topic is from our odom frame to our base frame.
  Eigen::Vector3d pose;
  calcTfPose(msg, &pose);

  if (!odom_integrator_ready_)
  {
    resetOdomIntegrator();
    odom_integrator_ready_ = true;
  }
  else
  {
    calcOdomDelta(pose);
  }
  odom_integrator_last_pose_ = pose;
}

void Node::calcTfPose(const nav_msgs::OdometryConstPtr& msg, Eigen::Vector3d* pose)
{
  tf2::Transform tf_pose;
  fromMsg(msg->pose.pose, tf_pose);
  (*pose)(0) = tf_pose.getOrigin().x();
  (*pose)(1) = tf_pose.getOrigin().y();
  double yaw, pitch, roll;
  tf_pose.getBasis().getEulerYPR(yaw, pitch, roll);
  (*pose)(2) = yaw;
}

void Node::calcOdomDelta(const Eigen::Vector3d& pose)
{
  Eigen::Vector3d delta;

  delta[0] = pose[0] - odom_integrator_last_pose_[0];
  delta[1] = pose[1] - odom_integrator_last_pose_[1];
  delta[2] = angles::shortest_angular_distance(odom_integrator_last_pose_[2], pose[2]);

  // project bearing change onto average orientation, x is forward translation, y is strafe
  double delta_trans, delta_rot, delta_bearing;
  delta_trans = std::sqrt(delta[0] * delta[0] + delta[1] * delta[1]);
  delta_rot = delta[2];
  if (delta_trans < 1e-6)
  {
    // For such a small translation, we either didn't move or rotated in place.
    // Assume the very small motion was forward, not strafe.
    delta_bearing = 0;
  }
  else
  {
    double angle_a = std::atan2(delta[1], delta[0]);
    double angle_b = odom_integrator_last_pose_[2] + delta_rot / 2;
    delta_bearing = angles::shortest_angular_distance(angle_b, angle_a);
  }
  double cs_bearing = std::cos(delta_bearing);
  double sn_bearing = std::sin(delta_bearing);

  // Accumulate absolute motion
  odom_integrator_absolute_motion_[0] += std::fabs(delta_trans * cs_bearing);
  odom_integrator_absolute_motion_[1] += std::fabs(delta_trans * sn_bearing);
  odom_integrator_absolute_motion_[2] += std::fabs(delta_rot);

  // We could also track velocity and acceleration here, for motion models that adjust for
  // velocity/acceleration. We could also track the covariance of the odometry message and
  // accumulate a total covariance across the time region for a motion model that uses the
  // reported covariance directly.
}

bool Node::getOdomPose(const ros::Time& t, Eigen::Vector3d* map_pose)
{
  // Get the robot's pose
  tf2::Stamped<tf2::Transform> ident;
  ident.setIdentity();
  geometry_msgs::TransformStamped ident_msg, latest_odom_pose_msg;
  ident_msg = tf2::toMsg(ident);
  try
  {
    geometry_msgs::TransformStamped stamped_tf = tf_buffer_.lookupTransform(odom_frame_id_, base_frame_id_,
                                                                            t, ros::Duration(0.5));
    tf2::doTransform(ident_msg, latest_odom_pose_msg, stamped_tf);
    tf2::fromMsg(latest_odom_pose_msg, latest_odom_pose_);
  }
  catch (tf2::TransformException e)
  {
    ROS_INFO_STREAM("Failed to compute odom pose, skipping scan (" << e.what() << ")");
    return false;
  }
  (*map_pose)(0) = latest_odom_pose_.getOrigin().x();
  (*map_pose)(1) = latest_odom_pose_.getOrigin().y();
  double pitch, roll, yaw;
  latest_odom_pose_.getBasis().getEulerYPR(yaw, pitch, roll);
  (*map_pose)(2) = yaw;
  return true;
}

// Helper function to generate a random free-space pose
Eigen::Vector3d Node::randomFreeSpacePose()
{
  Eigen::Vector3d p;
  if (free_space_indices_.size() == 0)
  {
    ROS_WARN("Free space indices have not been initialized");
    return p;
  }
  unsigned int rand_index = drand48() * free_space_indices_.size();
  std::pair<int, int> free_point = free_space_indices_.at(rand_index);
  std::vector<double> p_vec(2);
  map_->convertMapToWorld({ free_point.first, free_point.second }, &p_vec);
  p[0] = p_vec[0];
  p[1] = p_vec[1];
  p[2] = drand48() * 2 * M_PI - M_PI;
  return p;
}

// Helper function to score a pose for uniform pose generation
double Node::scorePose(const Eigen::Vector3d& p)
{
  return node_->scorePose(p);
}

Eigen::Vector3d Node::uniformPoseGenerator()
{
  double good_weight = uniform_pose_starting_weight_threshold_;
  const double deweight_multiplier = uniform_pose_deweight_multiplier_;
  Eigen::Vector3d p;
  p = randomFreeSpacePose();

  // Check and see how "good" this pose is.
  // Begin with the configured starting weight threshold,
  // then down-weight each try by the configured deweight multiplier.
  // A starting weight of 0 or negative means disable this check.
  // Also sanitize the value of deweight_multiplier.
  if (good_weight > 0.0 && deweight_multiplier < 1.0 && deweight_multiplier >= 0.0)
  {
    while (scorePose(p) < good_weight)
    {
      p = randomFreeSpacePose();
      good_weight *= deweight_multiplier;
    }
  }
  return p;
}

bool Node::globalLocalizationCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  if (map_ == NULL)
  {
    return true;
  }
  std::lock_guard<std::mutex> cfl(configuration_mutex_);
  global_localization_active_ = true;
  pf_->setDecayRates(global_localization_alpha_slow_, global_localization_alpha_fast_);
  node_->globalLocalizationCallback();
  pf_->initWithPoseFn(uniform_pose_generator_fn_);
  odom_init_ = false;
  return true;
}

void Node::publishTransform(const ros::TimerEvent& event)
{
  tf2::Transform tf_transform;
  if (tf_broadcast_ && getLatestTf(&tf_transform))
  {
    // We want to send a transform that is good up until a
    // tolerance time so that odom can be used
    ros::Time transform_expiration = (ros::Time::now() + transform_tolerance_);
    geometry_msgs::TransformStamped odom_to_map_msg_stamped;
    if (tf_reverse_)
    {
      odom_to_map_msg_stamped.header.frame_id = odom_frame_id_;
      odom_to_map_msg_stamped.child_frame_id = global_frame_id_;
    }
    else
    {
      odom_to_map_msg_stamped.header.frame_id = global_frame_id_;
      odom_to_map_msg_stamped.child_frame_id = odom_frame_id_;
      tf_transform = tf_transform.inverse();
    }
    odom_to_map_msg_stamped.header.stamp = transform_expiration;
    odom_to_map_msg_stamped.transform = tf2::toMsg(tf_transform);
    geometry_msgs::Quaternion quaternion = tf2::toMsg(tf_transform.getRotation());
    geometry_msgs::Vector3 origin = toMsg(tf_transform.getOrigin());
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = global_frame_id_;
    odom.child_frame_id = odom_frame_id_;
    odom.pose.pose.position.x = origin.x;
    odom.pose.pose.position.y = origin.y;
    odom.pose.pose.position.z = origin.z;
    odom.pose.pose.orientation = quaternion;
    map_odom_transform_pub_.publish(odom);
    tfb_.sendTransform(odom_to_map_msg_stamped);
    sent_first_transform_ = true;
  }
}

bool Node::getLatestTf(tf2::Transform* latest_tf)
{
  std::lock_guard<std::mutex> tfl(tf_mutex_);
  if(latest_tf_valid_)
  {
    *latest_tf = latest_tf_;
    return true;
  }
  return false;
}

void Node::getLatestPose(tf2::Transform latest_tf, geometry_msgs::PoseWithCovarianceStamped* latest_pose)
{
  std::lock_guard<std::mutex> lpl(latest_pose_mutex_);
  // We need to apply the last transform to the latest odom pose to get
  // the latest map pose to store.  We'll take the covariance from
  // last_published_pose_.
  tf2::Transform map_pose = latest_tf.inverse() * latest_odom_pose_;
  double yaw, pitch, roll;
  map_pose.getBasis().getEulerYPR(yaw, pitch, roll);
  geometry_msgs::Pose pose;
  pose = tf2::toMsg(map_pose, pose);
  latest_pose_.pose.pose = pose;
  latest_pose_.pose.covariance[COVARIANCE_XX] = last_published_pose_->pose.covariance[COVARIANCE_XX];
  latest_pose_.pose.covariance[COVARIANCE_YY] = last_published_pose_->pose.covariance[COVARIANCE_YY];
  latest_pose_.pose.covariance[COVARIANCE_AA] = last_published_pose_->pose.covariance[COVARIANCE_AA];
  latest_pose_.header.stamp = ros::Time::now();
  latest_pose_.header.frame_id = "map";
  *latest_pose = latest_pose_;
}

void Node::initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg_ptr)
{
  std::lock_guard<std::mutex> cfl(configuration_mutex_);
  geometry_msgs::PoseWithCovarianceStamped msg(*msg_ptr);
  resolveFrameId(msg);
  if(checkInitialPose(msg))
  {
    std::vector<double> cov_vals(36, 0.0);
    setCovarianceVals(msg, &cov_vals);
    tf2::Transform pose;
    transformMsgToTfPose(msg, &pose);
    setInitialPose(pose, cov_vals);
  }
}

void Node::setInitialPose(const tf2::Transform& pose, const std::vector<double>& covariance)
{
  setInitialPoseHyp(pose, covariance);
  applyInitialPose();
  // disable global localization in case it was active
  global_localization_active_ = false;
}

/**
 * If initial_pose_hyp_ and map_ are both non-null, apply the initial
 * pose to the particle filter state. Initial_pose_hyp_ is deleted
 * and set to NULL after it is used.
 */
void Node::applyInitialPose()
{
  if (initial_pose_hyp_ != NULL && map_ != NULL)
  {
    pf_->initWithGaussian(initial_pose_hyp_->mean, initial_pose_hyp_->covariance);
    odom_init_ = false;

    initial_pose_hyp_ = NULL;
  }
}

void Node::newInitialPoseSubscriber(const ros::SingleSubscriberPublisher& single_sub_pub)
{
  std::lock_guard<std::mutex> lpl(latest_pose_mutex_);
  if (latest_pose_.header.frame_id.compare("map") == 0)
  {
    ROS_INFO("New initial pose subscriber registered. Publishing latest amcl pose: (%f, %f).",
             latest_pose_.pose.pose.position.x, latest_pose_.pose.pose.position.y);
    single_sub_pub.publish(latest_pose_);
  }
  else
  {
    ROS_DEBUG("New initial pose subscriber registered. Latest amcl pose uninitialized, no pose will be published.");
  }
}

void Node::computeDelta(const Eigen::Vector3d& pose, Eigen::Vector3d* delta)
{
  // Compute change in pose
  (*delta)(0) = pose[0] - pf_odom_pose_[0];
  (*delta)(1) = pose[1] - pf_odom_pose_[1];
  (*delta)(2) = angles::shortest_angular_distance(pf_odom_pose_[2], pose[2]);
}

void Node::setScannersUpdateFlags(const Eigen::Vector3d& delta, std::vector<bool>& scanners_update, bool* force_update)
{
    // See if we should update the filter
    bool update;
    if (odom_integrator_enabled_)
    {
      double abs_trans = std::sqrt(odom_integrator_absolute_motion_[0] * odom_integrator_absolute_motion_[0]
                                   + odom_integrator_absolute_motion_[1] * odom_integrator_absolute_motion_[1]);
      double abs_rot = odom_integrator_absolute_motion_[2];
      update = abs_trans >= d_thresh_ || abs_rot >= a_thresh_;
    }
    else
    {
      update = std::fabs(delta[0]) > d_thresh_
                         || std::fabs(delta[1]) > d_thresh_
                         || std::fabs(delta[2]) > a_thresh_;
    }
    update = update || *force_update;
    *force_update = false;

    // Set the scanner update flags
    if (update)
      for (unsigned int i = 0; i < scanners_update.size(); i++)
        scanners_update.at(i) = true;
}

void Node::updateOdom(const Eigen::Vector3d& pose, const Eigen::Vector3d& delta)
{
  std::shared_ptr<OdomData> odata = std::make_shared<OdomData>();
  odata->pose = pose;
  // HACK
  // Modify the delta in the action data so the filter gets
  // updated correctly
  odata->delta = delta;
  odata->absolute_motion = odom_integrator_absolute_motion_;
  if (odom_integrator_enabled_)
  {
    geometry_msgs::Pose2D p;
    p.x = odata->absolute_motion[0];
    p.y = odata->absolute_motion[1];
    p.theta = odata->absolute_motion[2];
    absolute_motion_pub_.publish(p);
  }

  // Use the action data to update the filter
  odom_.updateAction(pf_, std::dynamic_pointer_cast<SensorData>(odata));
  resetOdomIntegrator();
  pf_odom_pose_ = pose;
}

void Node::initOdom(const Eigen::Vector3d& pose, std::vector<bool>& scanners_update,
                    int* resample_count, bool* force_publication)
{
  // Pose at last filter update
  pf_odom_pose_ = pose;
  // Filter is now initialized
  odom_init_ = true;
  // Should update sensor data
  for (unsigned int i = 0; i < scanners_update.size(); i++)
    scanners_update.at(i) = true;
  *force_publication = true;
  *resample_count = 0;
  initOdomIntegrator();
}

void Node::resolveFrameId(geometry_msgs::PoseWithCovarianceStamped& msg)
{
  // Rewrite to our global frame if received in the alt frame.
  // This allows us to run with multiple localizers using tf_reverse and pose them all at once.
  // And it is much cheaper to rewrite here than to run a separate topic tool transformer.
  if (msg.header.frame_id == global_alt_frame_id_)
  {
    msg.header.frame_id = global_frame_id_;
  }
}

bool Node::checkInitialPose(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
  if (msg.header.frame_id == "")
  {
    // This should be removed at some point
    ROS_WARN("Received initial pose with empty frame_id.  You should always supply a frame_id.");
    return false;
  }
  // We only accept initial pose estimates in the global frame, #5148.
  else if (msg.header.frame_id != global_frame_id_)
  {
    ROS_WARN_STREAM("Ignoring initial pose in frame \"" << msg.header.frame_id
                    << "\"; initial poses must be in the global frame, \"" << global_frame_id_ << "\"");
    return false;
  }

  geometry_msgs::Point position = msg.pose.pose.position;
  if (std::isnan(position.x) or std::isnan(position.y) or std::isnan(position.z))
  {
    ROS_WARN("Received initial pose with position value 'NAN'. Ignoring pose.");
    return false;
  }

  if (std::isnan(msg.pose.pose.orientation.x) or std::isnan(msg.pose.pose.orientation.y) or
      std::isnan(msg.pose.pose.orientation.z) or std::isnan(msg.pose.pose.orientation.w))
  {
    ROS_WARN("Received initial pose with orientation value 'NAN'. Ignoring pose.");
    return false;
  }
  return true;
}

void Node::setCovarianceVals(const geometry_msgs::PoseWithCovarianceStamped& msg, std::vector<double>* cov_vals)
{
  for (int i = 0; i < msg.pose.covariance.size(); i++)
  {
    if (std::isnan(msg.pose.covariance[i]))
    {
      cov_vals->at(i) = default_cov_vals_[i];
    }
    else
    {
      cov_vals->at(i) = msg.pose.covariance[i];
    }
  }
}

void Node::transformMsgToTfPose(const geometry_msgs::PoseWithCovarianceStamped& msg, tf2::Transform* pose)
{
  // In case the client sent us a pose estimate in the past, integrate the
  // intervening odometric change.
  tf2::Transform tx_odom;
  try
  {
    ros::Time now = ros::Time::now();
    // wait a little for the latest tf to become available
    geometry_msgs::TransformStamped tx_odom_msg = tf_buffer_.lookupTransform(base_frame_id_, msg.header.stamp,
                                                                             base_frame_id_, now, odom_frame_id_,
                                                                             ros::Duration(0.5));
    tf2::fromMsg(tx_odom_msg.transform, tx_odom);
  }
  catch (tf2::TransformException e)
  {
    // If we've never sent a transform, then this is normal, because the
    // global_frame_id_ frame doesn't exist.  We only care about in-time
    // transformation for on-the-move pose-setting, so ignoring this
    // startup condition doesn't really cost us anything.
    std::lock_guard<std::mutex> tfl(tf_mutex_);
    if (sent_first_transform_)
      ROS_WARN_STREAM("Failed to transform initial pose in time (" << e.what() << ")");
    tx_odom.setIdentity();
  }

  tf2::Transform pose_old;
  tf2::fromMsg(msg.pose.pose, pose_old);
  *pose = pose_old * tx_odom;
}

void Node::setInitialPoseHyp(const tf2::Transform& pose, const std::vector<double>& covariance)
{
  double roll, pitch, yaw;
  pose.getBasis().getRPY(roll, pitch, yaw);
  ROS_DEBUG("Setting pose (%.6f): %.3f %.3f %.3f", ros::Time::now().toSec(),
            pose.getOrigin().x(), pose.getOrigin().y(), yaw);
  ROS_INFO("Initial pose received by AMCL: (%.3f, %.3f)", pose.getOrigin().x(), pose.getOrigin().y());
  // Re-initialize the filter
  Eigen::Vector3d pf_init_pose_mean;
  pf_init_pose_mean[0] = pose.getOrigin().x();
  pf_init_pose_mean[1] = pose.getOrigin().y();
  pf_init_pose_mean[2] = yaw;
  Eigen::Matrix3d pf_init_pose_cov;
  // Copy in the covariance, converting from 6-D to 3-D
  for (int i = 0; i < 2; i++)
  {
    for (int j = 0; j < 2; j++)
    {
      pf_init_pose_cov(i, j) = covariance[6 * i + j];
    }
    pf_init_pose_cov(i, 2) = covariance[6 * i + 5];
    pf_init_pose_cov(2, i) = covariance[6 * 5 + i];
  }
  pf_init_pose_cov(2, 2) = covariance[6 * 5 + 5];
  initial_pose_hyp_ = std::make_shared<PoseHypothesis>();
  initial_pose_hyp_->mean = pf_init_pose_mean;
  initial_pose_hyp_->covariance = pf_init_pose_cov;
}

std::string Node::getOdomFrameId()
{
  return odom_frame_id_;
}

std::string Node::getBaseFrameId()
{
  return base_frame_id_;
}

}  // namespace amcl

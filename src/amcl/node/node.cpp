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
#include <tf/exceptions.h>
#include <tf/transform_datatypes.h>

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
    global_localization_active_(false),
    dsrv_(ros::NodeHandle("~"))
{
  std::lock_guard<std::mutex> cfl(configuration_mutex_);

  private_nh_.param("map_type", map_type_, 0);

  double tmp;
  private_nh_.param("transform_publish_rate", tmp, 50.0);
  transform_publish_period_ = ros::Duration(1.0 / tmp);
  private_nh_.param("save_pose_to_server_rate", tmp, 0.5);
  save_pose_to_server_period_ = ros::Duration(1.0 / tmp);
  private_nh_.param("save_pose_to_file_rate", tmp, 0.1);
  save_pose_to_file_period_ = ros::Duration(1.0 / tmp);

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

  private_nh_.param("save_pose", save_pose_, false);
  const std::string default_filepath = "badger_amcl_saved_pose.yaml";
  private_nh_.param("saved_pose_filepath", saved_pose_filepath_, default_filepath);

  std::string tmp_model_type;
  private_nh_.param("odom_model_type", tmp_model_type, std::string("diff"));
  if (tmp_model_type == "diff")
    odom_model_type_ = ODOM_MODEL_DIFF;
  else if (tmp_model_type == "omni")
    odom_model_type_ = ODOM_MODEL_OMNI;
  else if (tmp_model_type == "diff-corrected")
    odom_model_type_ = ODOM_MODEL_DIFF_CORRECTED;
  else if (tmp_model_type == "omni-corrected")
    odom_model_type_ = ODOM_MODEL_OMNI_CORRECTED;
  else if (tmp_model_type == "gaussian")
    odom_model_type_ = ODOM_MODEL_GAUSSIAN;
  else
  {
    ROS_WARN_STREAM("Unknown odom model type \"" << tmp_model_type << "\"; defaulting to diff model");
    odom_model_type_ = ODOM_MODEL_DIFF;
  }

  private_nh_.param("update_min_d", d_thresh_, 0.2);
  private_nh_.param("update_min_a", a_thresh_, M_PI / 6.0);
  private_nh_.param("odom_frame_id", odom_frame_id_, std::string("odom"));
  private_nh_.param("base_frame_id", base_frame_id_, std::string("base_link"));
  private_nh_.param("global_frame_id", global_frame_id_, std::string("map"));
  private_nh_.param("global_alt_frame_id", global_alt_frame_id_, std::string(""));
  private_nh_.param("resample_model_type", tmp_model_type, std::string("multinomial"));
  if (tmp_model_type == "multinomial")
    resample_model_type_ = PF_RESAMPLE_MULTINOMIAL;
  else if (tmp_model_type == "systematic")
    resample_model_type_ = PF_RESAMPLE_SYSTEMATIC;
  else
  {
    ROS_WARN_STREAM("Unknown resample model type \"" << tmp_model_type << "\"; defaulting to multinomial model");
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

  transform_tolerance_.fromSec(tmp_tol);

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

  publish_transform_timer_ = nh_.createTimer(transform_publish_period_, std::bind(&Node::publishTransform, this,
                                                                                  std::placeholders::_1));
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
  save_pose_to_server_period_ = ros::Duration(1.0 / config.save_pose_to_server_rate);
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
    ;
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
                                         uniform_pose_generator_fn_);
  pf_err_ = config.kld_err;
  pf_z_ = config.kld_z;
  pf_->setPopulationSizeParameters(pf_err_, pf_z_);
  pf_->setResampleModel(resample_model_type_);

  // Initialize the filter
  PFVector pf_init_pose_mean;
  pf_init_pose_mean.v[0] = last_published_pose_->pose.pose.position.x;
  pf_init_pose_mean.v[1] = last_published_pose_->pose.pose.position.y;
  pf_init_pose_mean.v[2] = tf::getYaw(last_published_pose_->pose.pose.orientation);
  PFMatrix pf_init_pose_cov;
  pf_init_pose_cov.m[0][0] = last_published_pose_->pose.covariance[COVARIANCE_XX];
  pf_init_pose_cov.m[1][1] = last_published_pose_->pose.covariance[COVARIANCE_YY];
  pf_init_pose_cov.m[2][2] = last_published_pose_->pose.covariance[COVARIANCE_AA];
  pf_->init(pf_init_pose_mean, pf_init_pose_cov);
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
  initial_pose_sub_ = nh_.subscribe("initialpose", 2, &Node::initialPoseReceived, this);
  publish_transform_timer_ = nh_.createTimer(transform_publish_period_, std::bind(&Node::publishTransform, this,
                                                                                  std::placeholders::_1));
}

void Node::setPfDecayRateNormal()
{
  pf_->setDecayRates(alpha_slow_, alpha_fast_);
}

bool Node::updatePf(const ros::Time& t, std::vector<bool>& scanners_update, int scanner_index,
                    int* resample_count, bool* force_publication, bool* force_update)
{
  // Where the robot was when this scan was taken
  PFVector pose;
  if (getOdomPose(t, &pose))
  {
    PFVector delta;
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
  for (int i = 0; i < set->sample_count; i++)
  {
    tf::poseTFToMsg(tf::Pose(tf::createQuaternionFromYaw(set->samples[i].pose.v[2]),
                             tf::Vector3(set->samples[i].pose.v[0], set->samples[i].pose.v[1], 0)),
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

void Node::updatePose(const PFVector& max_hyp_mean, const ros::Time& stamp)
{
  std::shared_ptr<geometry_msgs::PoseWithCovarianceStamped> p = (
          std::make_shared<geometry_msgs::PoseWithCovarianceStamped>());
  // Fill in the header
  p->header.frame_id = global_frame_id_;
  p->header.stamp = stamp;
  // Copy in the pose
  p->pose.pose.position.x = max_hyp_mean.v[0];
  p->pose.pose.position.y = max_hyp_mean.v[1];
  tf::quaternionTFToMsg(tf::createQuaternionFromYaw(max_hyp_mean.v[2]), p->pose.pose.orientation);
  // Copy in the covariance, converting from 3-D to 6-D
  std::shared_ptr<PFSampleSet> set = pf_->getCurrentSet();
  for (int i = 0; i < 2; i++)
  {
    for (int j = 0; j < 2; j++)
    {
      // Report the overall filter covariance, rather than the
      // covariance for the highest-weight cluster
      p->pose.covariance[6 * i + j] = set->cov.m[i][j];
    }
  }
  // Report the overall filter covariance, rather than the
  // covariance for the highest-weight cluster
  p->pose.covariance[COVARIANCE_AA] = set->cov.m[2][2];
  publishPose(*p);
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

bool Node::updateOdomToMapTransform(const tf::Stamped<tf::Pose>& odom_to_map)
{
  std::lock_guard<std::mutex> tfl(tf_mutex_);
  latest_tf_ = tf::Transform(tf::Quaternion(odom_to_map.getRotation()),
                             tf::Point(odom_to_map.getOrigin()));
  latest_tf_valid_ = false;

  try
  {
    latest_tf_ = tf::Transform(tf::Quaternion(odom_to_map.getRotation()), tf::Point(odom_to_map.getOrigin()));
    latest_tf_valid_ = true;
  }
  catch (tf::TransformException)
  {
    ROS_WARN("Failed to transform odom to map pose");
    return false;
  }
}

void Node::attemptSavePose()
{
  std::lock_guard<std::mutex> tfl(tf_mutex_);
  if (latest_tf_valid_)
  {
    // Is it time to save our last pose to the param server
    ros::Time now = ros::Time::now();
    if ((save_pose_to_server_period_.toSec() > 0.0)
        && (now - save_pose_to_server_last_time_) >= save_pose_to_server_period_)
    {
      savePoseToServer();
      save_pose_to_server_last_time_ = now;
    }
    if ((save_pose_to_file_period_.toSec() > 0.0)
        && (now - save_pose_to_file_last_time_) >= save_pose_to_file_period_)
    {
      ROS_DEBUG_STREAM("save pose to file period: " << save_pose_to_file_period_.toSec());
      savePoseToFile();
      save_pose_to_file_last_time_ = now;
    }
  }
}

void Node::loadPose()
{
  if (loadPoseFromServer())
  {
    ROS_INFO("Successfully loaded initial pose from server.");
    ROS_INFO("Pose loaded: (%.3f, %.3f)", init_pose_[0], init_pose_[1]);
  }
  else if (loadPoseFromFile())
  {
    ROS_INFO("Failed to load pose from server, but successfully loaded pose from file.");
    ROS_INFO("Pose loaded: (%.3f, %.3f)", init_pose_[0], init_pose_[1]);
  }
  else
  {
    ROS_WARN("Failed to load pose from server or file. Setting pose to default values.");
    init_pose_[0] = 0.0;
    init_pose_[1] = 0.0;
    init_pose_[2] = 0.0;
    init_cov_[0] = 0.5 * 0.5;
    init_cov_[1] = 0.5 * 0.5;
    init_cov_[2] = (M_PI / 12.0) * (M_PI / 12.0);
    ROS_INFO("Default pose: (%.3f, %.3f)", init_pose_[0], init_pose_[1]);
  }
}

void Node::publishInitialPoseInternal()
{
  geometry_msgs::PoseWithCovarianceStamped pose;
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = "/map";
  pose.pose.pose.position.x = init_pose_[0];
  pose.pose.pose.position.y = init_pose_[1];
  pose.pose.pose.position.z = 0.0;
  pose.pose.pose.orientation = tf::createQuaternionMsgFromYaw(init_pose_[2]);
  std::vector<double> cov_vals(36, 0.0);
  cov_vals[COVARIANCE_XX] = init_cov_[0];
  cov_vals[COVARIANCE_YY] = init_cov_[1];
  cov_vals[COVARIANCE_AA] = init_cov_[2];
  for (int i = 0; i < cov_vals.size(); i++)
  {
    pose.pose.covariance[i] = cov_vals[i];
  }
  ROS_INFO("Initial pose: (%.3f, %.3f)", pose.pose.pose.position.x, pose.pose.pose.position.y);
  initialPoseReceivedInternal(pose);
}

bool Node::loadPoseFromServer()
{
  bool success = loadParamFromServer("initial_pose_x", &init_pose_[0]);
  success = success and loadParamFromServer("initial_pose_y", &init_pose_[1]);
  success = success and loadParamFromServer("initial_pose_a", &init_pose_[2]);
  success = success and loadParamFromServer("initial_cov_xx", &init_cov_[0]);
  success = success and loadParamFromServer("initial_cov_yy", &init_cov_[1]);
  success = success and loadParamFromServer("initial_cov_aa", &init_cov_[2]);
  return success;
}

bool Node::loadParamFromServer(std::string param_name, double* val)
{
  double param_val;
  bool success = private_nh_.getParam(param_name, param_val) and !std::isnan(param_val);
  if(success)
  {
    *val = param_val;
  }
  else
  {
    ROS_DEBUG_STREAM("Failed to load " << param_name << " from server.");
  }
  return success;
}

bool Node::loadPoseFromFile()
{
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
    xx = config["pose"]["covariance"][COVARIANCE_XX].as<double>();
    yy = config["pose"]["covariance"][COVARIANCE_YY].as<double>();
    aa = config["pose"]["covariance"][COVARIANCE_AA].as<double>();
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM("Exception while loading pose from file. Failed to parse saved YAML pose. " << e.what());
    return false;
  }
  if (std::isnan(x) or std::isnan(y) or std::isnan(yaw) or std::isnan(xx) or std::isnan(yy) or std::isnan(aa))
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

// Caller must be holding tf_mutex_
void Node::savePoseToServer()
{
  if (!save_pose_)
  {
    ROS_DEBUG("As specified, not saving pose to server");
    return;
  }

  // We need to apply the last transform to the latest odom pose to get
  // the latest map pose to store.  We'll take the covariance from
  // last_published_pose_.
  tf::Pose map_pose = latest_tf_.inverse() * latest_odom_pose_;
  double yaw, pitch, roll;
  map_pose.getBasis().getEulerYPR(yaw, pitch, roll);

  private_nh_.setParam("initial_pose_x", map_pose.getOrigin().x());
  private_nh_.setParam("initial_pose_y", map_pose.getOrigin().y());
  private_nh_.setParam("initial_pose_a", yaw);
  private_nh_.setParam("initial_cov_xx", last_published_pose_->pose.covariance[COVARIANCE_XX]);
  private_nh_.setParam("initial_cov_yy", last_published_pose_->pose.covariance[COVARIANCE_YY]);
  private_nh_.setParam("initial_cov_aa", last_published_pose_->pose.covariance[COVARIANCE_AA]);
  geometry_msgs::Pose pose;
  tf::poseTFToMsg(map_pose, pose);
  std::lock_guard<std::mutex> lpl(latest_amcl_pose_mutex_);
  latest_amcl_pose_.pose.pose = pose;
  latest_amcl_pose_.pose.covariance[COVARIANCE_XX] = last_published_pose_->pose.covariance[COVARIANCE_XX];
  latest_amcl_pose_.pose.covariance[COVARIANCE_YY] = last_published_pose_->pose.covariance[COVARIANCE_YY];
  latest_amcl_pose_.pose.covariance[COVARIANCE_AA] = last_published_pose_->pose.covariance[COVARIANCE_AA];
  latest_amcl_pose_.header.stamp = ros::Time::now();
  latest_amcl_pose_.header.frame_id = "map";
}

void Node::savePoseToFile()
{
  if (!save_pose_)
  {
    ROS_DEBUG("As specified, not saving pose to file");
    return;
  }
  std::lock_guard<std::mutex> lpl(latest_amcl_pose_mutex_);

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
  covariance[COVARIANCE_XX] = latest_amcl_pose_.pose.covariance[COVARIANCE_XX];
  covariance[COVARIANCE_YY] = latest_amcl_pose_.pose.covariance[COVARIANCE_YY];
  covariance[COVARIANCE_AA] = latest_amcl_pose_.pose.covariance[COVARIANCE_AA];
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

void Node::initFromNewMap(std::shared_ptr<Map> new_map)
{
  map_ = new_map;
  // Create the particle filter
  uniform_pose_generator_fn_ = std::bind(&Node::uniformPoseGenerator, this);
  pf_ = std::make_shared<ParticleFilter>(min_particles_, max_particles_, alpha_slow_, alpha_fast_,
                                         uniform_pose_generator_fn_);
  pf_->setPopulationSizeParameters(pf_err_, pf_z_);
  pf_->setResampleModel(resample_model_type_);

  PFVector pf_init_pose_mean;
  pf_init_pose_mean.v[0] = init_pose_[0];
  pf_init_pose_mean.v[1] = init_pose_[1];
  pf_init_pose_mean.v[2] = init_pose_[2];
  PFMatrix pf_init_pose_cov;
  pf_init_pose_cov.m[0][0] = init_cov_[0];
  pf_init_pose_cov.m[1][1] = init_cov_[1];
  pf_init_pose_cov.m[2][2] = init_cov_[2];
  pf_->init(pf_init_pose_mean, pf_init_pose_cov);
  odom_init_ = false;

  // Instantiate the sensor objects
  // Odometry
  odom_.setModel(odom_model_type_, alpha1_, alpha2_, alpha3_, alpha4_, alpha5_);

  // Publish initial pose loaded from the server or file at startup
  publishInitialPoseInternal();
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
  odom_integrator_absolute_motion_ = PFVector();
}

void Node::integrateOdom(const nav_msgs::OdometryConstPtr& msg)
{
  // Integrate absolute motion relative to the base,
  // by finding the delta from one odometry message to another.
  // NOTE: assume this odom topic is from our odom frame to our base frame.
  PFVector pose;
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

void Node::calcTfPose(const nav_msgs::OdometryConstPtr& msg, PFVector* pose)
{
  tf::Pose tf_pose;
  poseMsgToTF(msg->pose.pose, tf_pose);
  pose->v[0] = tf_pose.getOrigin().x();
  pose->v[1] = tf_pose.getOrigin().y();
  double yaw, pitch, roll;
  tf_pose.getBasis().getEulerYPR(yaw, pitch, roll);
  pose->v[2] = yaw;
}

void Node::calcOdomDelta(const PFVector& pose)
{
  PFVector delta;

  delta.v[0] = pose.v[0] - odom_integrator_last_pose_.v[0];
  delta.v[1] = pose.v[1] - odom_integrator_last_pose_.v[1];
  delta.v[2] = angles::shortest_angular_distance(odom_integrator_last_pose_.v[2], pose.v[2]);

  // project bearing change onto average orientation, x is forward translation, y is strafe
  double delta_trans, delta_rot, delta_bearing;
  delta_trans = std::sqrt(delta.v[0] * delta.v[0] + delta.v[1] * delta.v[1]);
  delta_rot = delta.v[2];
  if (delta_trans < 1e-6)
  {
    // For such a small translation, we either didn't move or rotated in place.
    // Assume the very small motion was forward, not strafe.
    delta_bearing = 0;
  }
  else
  {
    double angle_a = std::atan2(delta.v[1], delta.v[0]);
    double angle_b = odom_integrator_last_pose_.v[2] + delta_rot / 2;
    delta_bearing = angles::shortest_angular_distance(angle_b, angle_a);
  }
  double cs_bearing = std::cos(delta_bearing);
  double sn_bearing = std::sin(delta_bearing);

  // Accumulate absolute motion
  odom_integrator_absolute_motion_.v[0] += std::fabs(delta_trans * cs_bearing);
  odom_integrator_absolute_motion_.v[1] += std::fabs(delta_trans * sn_bearing);
  odom_integrator_absolute_motion_.v[2] += std::fabs(delta_rot);

  // We could also track velocity and acceleration here, for motion models that adjust for
  // velocity/acceleration. We could also track the covariance of the odometry message and
  // accumulate a total covariance across the time region for a motion model that uses the
  // reported covariance directly.
}

bool Node::getOdomPose(const ros::Time& t, PFVector* map_pose)
{
  // Get the robot's pose
  tf::Stamped<tf::Pose> ident(tf::Transform(tf::createIdentityQuaternion(), tf::Vector3(0, 0, 0)), t, base_frame_id_);
  try
  {
    tf_.waitForTransform(base_frame_id_, odom_frame_id_, t, ros::Duration(0.5));
    tf_.transformPose(odom_frame_id_, ident, latest_odom_pose_);
  }
  catch (tf::TransformException e)
  {
    ROS_INFO_STREAM("Failed to compute odom pose, skipping scan (" << e.what() << ")");
    return false;
  }
  map_pose->v[0] = latest_odom_pose_.getOrigin().x();
  map_pose->v[1] = latest_odom_pose_.getOrigin().y();
  double pitch, roll, yaw;
  latest_odom_pose_.getBasis().getEulerYPR(yaw, pitch, roll);
  map_pose->v[2] = yaw;
  return true;
}

// Helper function to generate a random free-space pose
PFVector Node::randomFreeSpacePose()
{
  PFVector p;
  if (free_space_indices_.size() == 0)
  {
    ROS_WARN("Free space indices have not been initialized");
    return p;
  }
  unsigned int rand_index = drand48() * free_space_indices_.size();
  std::pair<int, int> free_point = free_space_indices_.at(rand_index);
  std::vector<double> p_vec(2);
  map_->convertMapToWorld({ free_point.first, free_point.second }, &p_vec);
  p.v[0] = p_vec[0];
  p.v[1] = p_vec[1];
  p.v[2] = drand48() * 2 * M_PI - M_PI;
  return p;
}

// Helper function to score a pose for uniform pose generation
double Node::scorePose(const PFVector& p)
{
  return node_->scorePose(p);
}

PFVector Node::uniformPoseGenerator()
{
  double good_weight = uniform_pose_starting_weight_threshold_;
  const double deweight_multiplier = uniform_pose_deweight_multiplier_;
  PFVector p;
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
  pf_->initModel(uniform_pose_generator_fn_);
  odom_init_ = false;
  return true;
}

void Node::publishTransform(const ros::TimerEvent& event)
{
  std::lock_guard<std::mutex> tfl(tf_mutex_);
  if (tf_broadcast_ && latest_tf_valid_)
  {
    // We want to send a transform that is good up until a
    // tolerance time so that odom can be used
    ros::Time transform_expiration = (ros::Time::now() + transform_tolerance_);
    tf::StampedTransform tmp_tf_stamped;
    tf::Transform tf_transform;
    if (tf_reverse_)
    {
      tmp_tf_stamped = tf::StampedTransform(latest_tf_, transform_expiration, odom_frame_id_, global_frame_id_);
      tf_transform = latest_tf_;
    }
    else
    {
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
    tfb_.sendTransform(tmp_tf_stamped);
    sent_first_transform_ = true;
  }
}

void Node::initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg_ptr)
{
  std::lock_guard<std::mutex> cfl(configuration_mutex_);
  geometry_msgs::PoseWithCovarianceStamped msg(*msg_ptr);
  initialPoseReceivedInternal(msg);
}

void Node::initialPoseReceivedInternal(geometry_msgs::PoseWithCovarianceStamped& msg)
{
  resolveFrameId(msg);
  if(checkInitialPose(msg))
  {
    handleInitialPose(msg);
  }
}

void Node::handleInitialPose(geometry_msgs::PoseWithCovarianceStamped& msg)
{
  setMsgCovarianceVals(&msg);
  tf::Pose pose;
  transformMsgToTfPose(msg, &pose);
  transformPoseToGlobalFrame(msg, pose);
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
    pf_->init(initial_pose_hyp_->mean, initial_pose_hyp_->covariance);
    odom_init_ = false;

    initial_pose_hyp_ = NULL;
  }
}

void Node::newInitialPoseSubscriber(const ros::SingleSubscriberPublisher& single_sub_pub)
{
  std::lock_guard<std::mutex> lpl(latest_amcl_pose_mutex_);
  if (latest_amcl_pose_.header.frame_id.compare("map") == 0)
  {
    ROS_INFO("New initial pose subscriber registered. Publishing latest amcl pose: (%f, %f).",
             latest_amcl_pose_.pose.pose.position.x, latest_amcl_pose_.pose.pose.position.y);
    single_sub_pub.publish(latest_amcl_pose_);
  }
  else
  {
    ROS_DEBUG("New initial pose subscriber registered. Latest amcl pose uninitialized, no pose will be published.");
  }
}

void Node::computeDelta(const PFVector& pose, PFVector* delta)
{
  // Compute change in pose
  delta->v[0] = pose.v[0] - pf_odom_pose_.v[0];
  delta->v[1] = pose.v[1] - pf_odom_pose_.v[1];
  delta->v[2] = angles::shortest_angular_distance(pf_odom_pose_.v[2], pose.v[2]);
}

void Node::setScannersUpdateFlags(const PFVector& delta, std::vector<bool>& scanners_update, bool* force_update)
{
    // See if we should update the filter
    bool update;
    if (odom_integrator_enabled_)
    {
      double abs_trans = std::sqrt(odom_integrator_absolute_motion_.v[0] * odom_integrator_absolute_motion_.v[0]
                                   + odom_integrator_absolute_motion_.v[1] * odom_integrator_absolute_motion_.v[1]);
      double abs_rot = odom_integrator_absolute_motion_.v[2];
      update = abs_trans >= d_thresh_ || abs_rot >= a_thresh_;
    }
    else
    {
      update = std::fabs(delta.v[0]) > d_thresh_
                         || std::fabs(delta.v[1]) > d_thresh_
                         || std::fabs(delta.v[2]) > a_thresh_;
    }
    update = update || *force_update;
    *force_update = false;

    // Set the scanner update flags
    if (update)
      for (unsigned int i = 0; i < scanners_update.size(); i++)
        scanners_update.at(i) = true;
}

void Node::updateOdom(const PFVector& pose, const PFVector &delta)
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
    p.x = odata->absolute_motion.v[0];
    p.y = odata->absolute_motion.v[1];
    p.theta = odata->absolute_motion.v[2];
    absolute_motion_pub_.publish(p);
  }

  // Use the action data to update the filter
  odom_.updateAction(pf_, std::dynamic_pointer_cast<SensorData>(odata));
  resetOdomIntegrator();
  pf_odom_pose_ = pose;
}

void Node::initOdom(const PFVector& pose, std::vector<bool>& scanners_update,
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
  if (tf_.resolve(msg.header.frame_id) == tf_.resolve(global_alt_frame_id_))
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
  else if (tf_.resolve(msg.header.frame_id) != tf_.resolve(global_frame_id_))
  {
    ROS_WARN_STREAM("Ignoring initial pose in frame \"" << msg.header.frame_id
                    << "\"; initial poses must be in the global frame, \"" << global_frame_id_ << "\"");
    return false;
  }

  if (std::isnan(msg.pose.pose.position.x) or std::isnan(msg.pose.pose.position.y) or std::isnan(msg.pose.pose.position.z))
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

void Node::setMsgCovarianceVals(geometry_msgs::PoseWithCovarianceStamped* msg)
{
  std::vector<double> default_cov_vals(36, 0.0);
  default_cov_vals[COVARIANCE_XX] = 0.5 * 0.5;
  default_cov_vals[COVARIANCE_YY] = 0.5 * 0.5;
  default_cov_vals[COVARIANCE_AA] = (M_PI / 12.0) * (M_PI / 12.0);
  for (int i = 0; i < msg->pose.covariance.size(); i++)
  {
    if (std::isnan(msg->pose.covariance[i]))
    {
      msg->pose.covariance[i] = default_cov_vals[i];
    }
  }
}

void Node::transformMsgToTfPose(const geometry_msgs::PoseWithCovarianceStamped& msg,
                                tf::Pose* pose)
{
  // In case the client sent us a pose estimate in the past, integrate the
  // intervening odometric change.
  tf::StampedTransform tx_odom;
  try
  {
    ros::Time now = ros::Time::now();
    // wait a little for the latest tf to become available
    tf_.waitForTransform(base_frame_id_, msg.header.stamp, base_frame_id_, now, odom_frame_id_, ros::Duration(0.5));
    tf_.lookupTransform(base_frame_id_, msg.header.stamp, base_frame_id_, now, odom_frame_id_, tx_odom);
  }
  catch (tf::TransformException e)
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

  tf::Pose pose_old;
  tf::poseMsgToTF(msg.pose.pose, pose_old);
  *pose = pose_old * tx_odom;
}

void Node::transformPoseToGlobalFrame(const geometry_msgs::PoseWithCovarianceStamped& msg, const tf::Pose& pose)
{
  ROS_DEBUG("Setting pose (%.6f): %.3f %.3f %.3f", ros::Time::now().toSec(),
            pose.getOrigin().x(), pose.getOrigin().y(), getYaw(pose));
  ROS_INFO("Initial pose received by AMCL: (%.3f, %.3f)", pose.getOrigin().x(), pose.getOrigin().y());
  // Re-initialize the filter
  PFVector pf_init_pose_mean;
  pf_init_pose_mean.v[0] = pose.getOrigin().x();
  pf_init_pose_mean.v[1] = pose.getOrigin().y();
  pf_init_pose_mean.v[2] = getYaw(pose);
  PFMatrix pf_init_pose_cov;
  // Copy in the covariance, converting from 6-D to 3-D
  for (int i = 0; i < 2; i++)
  {
    for (int j = 0; j < 2; j++)
    {
      pf_init_pose_cov.m[i][j] = msg.pose.covariance[6 * i + j];
    }
  }
  pf_init_pose_cov.m[2][2] = msg.pose.covariance[6 * 5 + 5];
  initial_pose_hyp_ = std::make_shared<PoseHypothesis>();
  initial_pose_hyp_->mean = pf_init_pose_mean;
  initial_pose_hyp_->covariance = pf_init_pose_cov;
}

double Node::getYaw(const tf::Pose& t)
{
  double yaw, pitch, roll;
  t.getBasis().getEulerYPR(yaw, pitch, roll);
  return yaw;
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

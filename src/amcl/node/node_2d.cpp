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

#include "node/node_2d.h"

#include <functional>

#include <angles/angles.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose2D.h>
#include <ros/assert.h>
#include <ros/console.h>
#include <ros/names.h>
#include <tf2/exceptions.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "node/node.h"

namespace badger_amcl
{

Node2D::Node2D(Node* node, std::mutex& configuration_mutex)
    : node_(node),
      configuration_mutex_(configuration_mutex),
      private_nh_("~"),
      resample_count_(0),
      tf_listener_(tf_buffer_)
{
  map_ = nullptr;
  latest_scan_data_ = NULL;
  fake_sample_set_ = std::make_shared<PFSampleSet>();
  private_nh_.param("first_map_only", first_map_only_, false);
  private_nh_.param("laser_min_range", sensor_min_range_, -1.0);
  private_nh_.param("laser_max_range", sensor_max_range_, -1.0);
  private_nh_.param("laser_max_beams", max_beams_, 30);
  private_nh_.param("laser_z_hit", z_hit_, 0.95);
  private_nh_.param("laser_z_short", z_short_, 0.1);
  private_nh_.param("laser_z_max", z_max_, 0.05);
  private_nh_.param("laser_z_rand", z_rand_, 0.05);
  private_nh_.param("laser_sigma_hit", sigma_hit_, 0.2);
  private_nh_.param("laser_lambda_short", lambda_short_, 0.1);
  private_nh_.param("laser_likelihood_max_dist", sensor_likelihood_max_dist_, 2.0);
  private_nh_.param("laser_gompertz_a", gompertz_a_, 1.0);
  private_nh_.param("laser_gompertz_b", gompertz_b_, 1.0);
  private_nh_.param("laser_gompertz_c", gompertz_c_, 1.0);
  private_nh_.param("laser_gompertz_input_shift", gompertz_input_shift_, 0.0);
  private_nh_.param("laser_gompertz_input_scale", gompertz_input_scale_, 1.0);
  private_nh_.param("laser_gompertz_output_shift", gompertz_output_shift_, 0.0);
  private_nh_.param("laser_scanner_off_map_factor", off_map_factor_, 1.0);
  private_nh_.param("laser_scanner_non_free_space_factor", non_free_space_factor_, 1.0);
  private_nh_.param("laser_scanner_non_free_space_radius", non_free_space_radius_, 0.0);
  private_nh_.param("resample_interval", resample_interval_, 2);
  private_nh_.param("do_beamskip", do_beamskip_, false);
  private_nh_.param("beam_skip_distance", beam_skip_distance_, 0.5);
  private_nh_.param("beam_skip_threshold", beam_skip_threshold_, 0.3);
  private_nh_.param("beam_skip_error_threshold_", beam_skip_error_threshold_, 0.9);
  private_nh_.param("global_localization_planar_off_map_factor", global_localization_off_map_factor_, 1.0);
  private_nh_.param("global_localization_planar_non_free_space_factor",
                    global_localization_non_free_space_factor_, 1.0);

  std::string model_type_str;
  private_nh_.param("laser_model_type", model_type_str, std::string("likelihood_field"));
  if (model_type_str == "beam")
    model_type_ = PLANAR_MODEL_BEAM;
  else if (model_type_str == "likelihood_field")
    model_type_ = PLANAR_MODEL_LIKELIHOOD_FIELD;
  else if (model_type_str == "likelihood_field_prob")
    model_type_ = PLANAR_MODEL_LIKELIHOOD_FIELD_PROB;
  else if (model_type_str == "likelihood_field_gompertz")
    model_type_ = PLANAR_MODEL_LIKELIHOOD_FIELD_GOMPERTZ;
  else
  {
    ROS_WARN_STREAM("Unknown planar model type \"" << model_type_str << "\"; defaulting to likelihood_field model");
    model_type_ = PLANAR_MODEL_LIKELIHOOD_FIELD;
  }
  private_nh_.param("map_scale_up_factor", map_scale_up_factor_, 1);
  // Prevent nonsense and crashes due to wacky values
  if (map_scale_up_factor_ < 1)
    map_scale_up_factor_ = 1;
  if (map_scale_up_factor_ > 16)
    map_scale_up_factor_ = 16;

  scan_topic_ = "scan";
  scan_sub_ = std::unique_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>>(
      new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, scan_topic_, 1));
  scan_filter_ = std::unique_ptr<tf2_ros::MessageFilter<sensor_msgs::LaserScan>>(
      new tf2_ros::MessageFilter<sensor_msgs::LaserScan>(*scan_sub_.get(), tf_buffer_, node_->getOdomFrameId(), 1, nh_));
  scan_filter_->registerCallback(std::bind(&Node2D::scanReceived, this, std::placeholders::_1));

  // 15s timer to warn on lack of receipt of planar scans, #5209
  check_scanner_interval_ = ros::Duration(15.0);
  check_scanner_timer_ = nh_.createTimer(check_scanner_interval_, std::bind(&Node2D::checkScanReceived, this,
                                                                            std::placeholders::_1));

  force_update_ = false;
  first_map_received_ = false;
  map_sub_ = nh_.subscribe("map", 1, &Node2D::mapMsgReceived, this);
}

Node2D::~Node2D()
{
  // TF message filters must be destroyed before the underlying subsriber.
  scan_filter_.reset();
}

void Node2D::reconfigure(AMCLConfig& config)
{
  sensor_min_range_ = config.laser_min_range;
  sensor_max_range_ = config.laser_max_range;
  z_hit_ = config.laser_z_hit;
  z_short_ = config.laser_z_short;
  z_max_ = config.laser_z_max;
  z_rand_ = config.laser_z_rand;
  sigma_hit_ = config.laser_sigma_hit;
  lambda_short_ = config.laser_lambda_short;
  sensor_likelihood_max_dist_ = config.laser_likelihood_max_dist;
  off_map_factor_ = config.laser_off_map_factor;
  non_free_space_factor_ = config.laser_non_free_space_factor;
  non_free_space_radius_ = config.laser_non_free_space_radius;
  global_localization_off_map_factor_ = config.global_localization_laser_off_map_factor;
  global_localization_non_free_space_factor_ = config.global_localization_laser_non_free_space_factor;
  resample_interval_ = config.resample_interval;
  do_beamskip_ = config.do_beamskip;
  beam_skip_distance_ = config.beam_skip_distance;
  beam_skip_threshold_ = config.beam_skip_threshold;
  gompertz_a_ = config.laser_gompertz_a;
  gompertz_b_ = config.laser_gompertz_b;
  gompertz_c_ = config.laser_gompertz_c;
  gompertz_input_shift_ = config.laser_gompertz_input_shift;
  gompertz_input_scale_ = config.laser_gompertz_input_scale;
  gompertz_output_shift_ = config.laser_gompertz_output_shift;
  max_beams_ = config.laser_max_beams;
  if (config.laser_model_type == "beam")
    model_type_ = PLANAR_MODEL_BEAM;
  else if (config.laser_model_type == "likelihood_field")
    model_type_ = PLANAR_MODEL_LIKELIHOOD_FIELD;
  else if (config.laser_model_type == "likelihood_field_prob")
    model_type_ = PLANAR_MODEL_LIKELIHOOD_FIELD_PROB;
  else if (config.laser_model_type == "likelihood_field_gompertz")
    model_type_ = PLANAR_MODEL_LIKELIHOOD_FIELD_GOMPERTZ;
  scanner_.init(max_beams_, map_);
  if (model_type_ == PLANAR_MODEL_BEAM)
    scanner_.setModelBeam(z_hit_, z_short_, z_max_, z_rand_, sigma_hit_, lambda_short_);
  else if (model_type_ == PLANAR_MODEL_LIKELIHOOD_FIELD_PROB)
  {
    ROS_INFO("Initializing likelihood field model; this can take some time on large maps...");
    scanner_.setModelLikelihoodFieldProb(z_hit_, z_rand_, sigma_hit_, sensor_likelihood_max_dist_,
                                         do_beamskip_, beam_skip_distance_, beam_skip_threshold_,
                                         beam_skip_error_threshold_);
    ROS_INFO("Done initializing likelihood field model with probabilities.");
  }
  else if (model_type_ == PLANAR_MODEL_LIKELIHOOD_FIELD)
  {
    ROS_INFO("Initializing likelihood field model; this can take some time on large maps...");
    scanner_.setModelLikelihoodField(z_hit_, z_rand_, sigma_hit_, sensor_likelihood_max_dist_);
    ROS_INFO("Done initializing likelihood field model.");
  }
  else if (model_type_ == PLANAR_MODEL_LIKELIHOOD_FIELD_GOMPERTZ)
  {
    ROS_INFO("Initializing likelihood field (gompertz) model; this can take some time on large maps...");
    scanner_.setModelLikelihoodFieldGompertz(
        z_hit_, z_rand_, sigma_hit_, sensor_likelihood_max_dist_, gompertz_a_, gompertz_b_,
        gompertz_c_, gompertz_input_shift_, gompertz_input_scale_, gompertz_output_shift_);
    ROS_INFO("Gompertz key points by total planar scan match: 0.0: %f, 0.25: %f, 0.5: %f, 0.75: %f, 1.0: %f",
             scanner_.applyGompertz(z_rand_),
             scanner_.applyGompertz(z_rand_ + z_hit_ * .25),
             scanner_.applyGompertz(z_rand_ + z_hit_ * .5),
             scanner_.applyGompertz(z_rand_ + z_hit_ * .75),
             scanner_.applyGompertz(z_rand_ + z_hit_));
    ROS_INFO("Done initializing likelihood (gompertz) field model.");
  }
  scanner_.setMapFactors(off_map_factor_, non_free_space_factor_, non_free_space_radius_);

  scan_filter_.reset();
  scan_sub_.reset();

  scan_sub_.reset(new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, scan_topic_, 1));
  scan_filter_.reset(new tf2_ros::MessageFilter<sensor_msgs::LaserScan>(*scan_sub_.get(), tf_buffer_,
                                                                        node_->getOdomFrameId(), 1, nh_));

  scan_filter_->registerCallback(std::bind(&Node2D::scanReceived, this, std::placeholders::_1));
  pf_ = node_->getPfPtr();
}

void Node2D::mapMsgReceived(const nav_msgs::OccupancyGridConstPtr& msg)
{
  if (first_map_only_ && first_map_received_)
  {
    ROS_DEBUG("Occupancy map already received");
    return;
  }

  std::lock_guard<std::mutex> cfl(configuration_mutex_);
  ROS_INFO("Received a %d X %d occupancy map @ %.3f m/pix\n", msg->info.width, msg->info.height, msg->info.resolution);
  map_ = convertMap(*msg);
  // Clear queued planar scanner objects because they hold pointers to the existing map
  scanners_.clear();
  scanners_update_.clear();
  frame_to_scanner_.clear();
  latest_scan_data_ = NULL;
  initFromNewMap();
  updateFreeSpaceIndices();
  first_map_received_ = true;
}

void Node2D::initFromNewMap()
{
  scanner_.init(max_beams_, map_);
  if (model_type_ == PLANAR_MODEL_BEAM)
    scanner_.setModelBeam(z_hit_, z_short_, z_max_, z_rand_, sigma_hit_, lambda_short_);
  else if (model_type_ == PLANAR_MODEL_LIKELIHOOD_FIELD_PROB)
  {
    ROS_INFO("Initializing likelihood field model; this can take some time on large maps...");
    scanner_.setModelLikelihoodFieldProb(z_hit_, z_rand_, sigma_hit_, sensor_likelihood_max_dist_,
                                         do_beamskip_, beam_skip_distance_, beam_skip_threshold_,
                                         beam_skip_error_threshold_);
    ROS_INFO("Done initializing likelihood field model.");
  }
  else if (model_type_ == PLANAR_MODEL_LIKELIHOOD_FIELD_GOMPERTZ)
  {
    ROS_INFO("Initializing likelihood field (gompertz) model; this can take some time on large maps...");
    scanner_.setModelLikelihoodFieldGompertz(
        z_hit_, z_rand_, sigma_hit_, sensor_likelihood_max_dist_, gompertz_a_, gompertz_b_,
        gompertz_c_, gompertz_input_shift_, gompertz_input_scale_, gompertz_output_shift_);
    ROS_INFO("Gompertz key points by total planar scan match: 0.0: %f, 0.25: %f, 0.5: %f, 0.75: %f, 1.0: %f",
             scanner_.applyGompertz(z_rand_),
             scanner_.applyGompertz(z_rand_ + z_hit_ * .25),
             scanner_.applyGompertz(z_rand_ + z_hit_ * .5),
             scanner_.applyGompertz(z_rand_ + z_hit_ * .75),
             scanner_.applyGompertz(z_rand_ + z_hit_));
    ROS_INFO("Done initializing likelihood (gompertz) field model.");
  }
  else
  {
    ROS_INFO("Initializing likelihood field model; this can take some time on large maps...");
    scanner_.setModelLikelihoodField(z_hit_, z_rand_, sigma_hit_, sensor_likelihood_max_dist_);
    ROS_INFO("Done initializing likelihood field model.");
  }
  scanner_.setMapFactors(off_map_factor_, non_free_space_factor_, non_free_space_radius_);
  node_->initFromNewMap(map_, not first_map_received_);
  pf_ = node_->getPfPtr();
}

/**
 * Convert an OccupancyGrid map message into the internal
 * representation.  This allocates an OccupancyMap and returns it.
 */
std::shared_ptr<OccupancyMap> Node2D::convertMap(const nav_msgs::OccupancyGrid& map_msg)
{
  double resolution = map_msg.info.resolution / map_scale_up_factor_;
  std::shared_ptr<OccupancyMap> occupancy_map = std::make_shared<OccupancyMap>(resolution);
  ROS_ASSERT(occupancy_map);
  std::vector<int> size_vec;
  size_vec.push_back(map_msg.info.width * map_scale_up_factor_);
  size_vec.push_back(map_msg.info.height * map_scale_up_factor_);
  occupancy_map->setSize(size_vec);
  double x_origin, y_origin;
  x_origin = map_msg.info.origin.position.x + (size_vec[0] / 2) * resolution;
  y_origin = map_msg.info.origin.position.y + (size_vec[1] / 2) * resolution;
  occupancy_map->setOrigin(pcl::PointXYZ(x_origin, y_origin, 0.0));
  for (int y = 0; y < size_vec[1]; y++)
  {
    int i = y * size_vec[0];
    const int msg_row = (y / map_scale_up_factor_) * (map_msg.info.width);
    for (int x = 0; x < size_vec[0]; x++, i++)
    {
      const int msg_i = msg_row + x / map_scale_up_factor_;

      if (map_msg.data[msg_i] == 0)
        occupancy_map->setCellState(i, MapCellState::CELL_FREE);
      else if (map_msg.data[msg_i] == 100)
        occupancy_map->setCellState(i, MapCellState::CELL_OCCUPIED);
      else
        occupancy_map->setCellState(i, MapCellState::CELL_UNKNOWN);
    }
  }
  return occupancy_map;
}

// Helper function to score a pose for uniform pose generation
double Node2D::scorePose(const Eigen::Vector3d& p)
{
  // There is no data to match, so return a perfect match
  double score = 1.0;
  if (latest_scan_data_ != NULL)
  {
    // Create a fake "sample set" of just this pose to score it.
    fake_sample_.pose[0] = p[0];
    fake_sample_.pose[1] = p[1];
    fake_sample_.pose[2] = p[2];
    fake_sample_.weight = 1.0;
    fake_sample_set_->sample_count = 1;
    fake_sample_set_->samples = { fake_sample_ };
    fake_sample_set_->converged = 0;
    scanner_.applyModelToSampleSet(latest_scan_data_, fake_sample_set_);
    score = fake_sample_.weight;
  }
  return score;
}

void Node2D::updateFreeSpaceIndices()
{
  // Index of free space
  // Must be calculated after the distances lut is set by the planar model
  std::vector<std::pair<int, int>> fsi;
  std::vector<int> size_vec = map_->getSize();
  for (int i = 0; i < size_vec[0]; i++)
  {
    for (int j = 0; j < size_vec[1]; j++)
    {
      if (map_->getCellState(i, j) == MapCellState::CELL_FREE)
      {
        if (map_->getDistanceToObject(i, j) > non_free_space_radius_)
        {
          fsi.push_back(std::make_pair(i, j));
        }
      }
    }
  }
  node_->updateFreeSpaceIndices(fsi);
}

void Node2D::scanReceived(const sensor_msgs::LaserScanConstPtr& planar_scan)
{
  latest_scan_received_ts_ = ros::Time::now();
  if(!isMapInitialized())
    return;

  if(!global_localization_active_)
    deactivateGlobalLocalizationParams();

  ros::Time stamp = planar_scan->header.stamp;
  int scanner_index = getFrameToScannerIndex(planar_scan->header.frame_id);
  if(scanner_index >= 0)
  {
    bool force_publication = false, resampled = false, success;
    success = updateNodePf(stamp, scanner_index, &force_publication);
    if(scanners_update_.at(scanner_index))
      success = success and updateScanner(planar_scan, scanner_index, &resampled);
    if(force_publication or resampled)
      success = success and resamplePose(stamp);
    if(success)
      node_->attemptSavePose(false);
  }
}

bool Node2D::updateNodePf(const ros::Time& stamp, int scanner_index, bool* force_publication)
{
  return node_->updatePf(stamp, scanners_update_, scanner_index, &resample_count_, force_publication, &force_update_);
}

bool Node2D::updateScanner(const sensor_msgs::LaserScanConstPtr& planar_scan,
                           int scanner_index, bool* resampled)
{
  initLatestScanData(planar_scan, scanner_index);
  double angle_min, angle_increment;
  bool success = true;
  if(getAngleStats(planar_scan, &angle_min, &angle_increment))
  {
    ROS_DEBUG("Planar scanner %d angles in base frame: min: %.3f inc: %.3f", scanner_index, angle_min, angle_increment);
    updateLatestScanData(planar_scan, angle_min, angle_increment);
    scanners_[scanner_index]->updateSensor(pf_, std::dynamic_pointer_cast<SensorData>(latest_scan_data_));
    scanners_update_.at(scanner_index) = false;
    if(!(++resample_count_ % resample_interval_))
    {
      resampleParticles();
      *resampled = true;
    }
    if(!force_update_)
       node_->publishParticleCloud();
  }
  else
  {
    success = false;
  }
  return success;
}

bool Node2D::isMapInitialized()
{
  if (map_ == NULL)
  {
    ROS_DEBUG("Map is null");
    return false;
  }
  if (pf_ == NULL)
  {
    ROS_DEBUG("PF is null");
    return false;
  }
  if (not map_->isDistancesLUTCreated())
  {
    ROS_DEBUG("Distances not yet created");
    return false;
  }
  return true;
}

void Node2D::deactivateGlobalLocalizationParams()
{
  std::lock_guard<std::mutex> cfl(configuration_mutex_);
  // Handle corner cases like getting dynamically reconfigured or getting a
  // new map by de-activating the global localization parameters here if we are
  // no longer globally localizing.
  node_->setPfDecayRateNormal();
  scanner_.setMapFactors(off_map_factor_, non_free_space_factor_, non_free_space_radius_);
  for (auto& l : scanners_)
  {
    l->setMapFactors(off_map_factor_, non_free_space_factor_, non_free_space_radius_);
  }
}

int Node2D::getFrameToScannerIndex(const std::string& scanner_frame_id)
{
  int scanner_index;
  // Do we have the base->base_laser Tx yet?
  if (frame_to_scanner_.find(scanner_frame_id) == frame_to_scanner_.end())
  {
    tf2::Transform scanner_pose;
    initFrameToScanner(scanner_frame_id, &scanner_pose, &scanner_index);
    if(scanner_index >= 0)
    {
      frame_to_scanner_[scanner_frame_id] = scanner_index;
      updateScannerPose(scanner_pose, scanner_index);
    }
  }
  else
  {
    // we have the planar scanner pose, retrieve planar scanner index
    scanner_index = frame_to_scanner_[scanner_frame_id];
  }
  return scanner_index;
}

bool Node2D::initFrameToScanner(const std::string& scanner_frame_id, tf2::Transform* scanner_pose, int* scanner_index)
{
  ROS_DEBUG_STREAM("Setting up planar_scanner " << frame_to_scanner_.size()
                   << " (scanner_frame_id=" << scanner_frame_id << ")");
  scanners_.push_back(std::make_shared<PlanarScanner>(scanner_));
  scanners_update_.push_back(true);
  *scanner_index = frame_to_scanner_.size();

  geometry_msgs::Pose ident, scanner_pose_msg;
  tf2::Transform ident_tf;
  ident_tf.setIdentity();
  ident = tf2::toMsg(ident_tf, ident);
  try
  {
    geometry_msgs::TransformStamped t = tf_buffer_.lookupTransform(node_->getBaseFrameId(), scanner_frame_id,
                                                                   ros::Time::now(), ros::Duration(5.0));
    tf2::doTransform(ident, scanner_pose_msg, t);
    tf2::fromMsg(scanner_pose_msg, *scanner_pose);
  }
  catch (tf2::TransformException& e)
  {
    ROS_ERROR_STREAM("Couldn't transform from " << scanner_frame_id << " to " << node_->getBaseFrameId()
                     << ", even though the message notifier is in use");
    return false;
  }
  return true;
}

void Node2D::updateScannerPose(const tf2::Transform& scanner_pose, int scanner_index)
{
  Eigen::Vector3d scanner_pose_v;
  scanner_pose_v[0] = scanner_pose.getOrigin().x();
  scanner_pose_v[1] = scanner_pose.getOrigin().y();
  // planar scanner mounting angle gets computed later -> set to 0 here!
  scanner_pose_v[2] = 0;
  scanners_[scanner_index]->setPlanarScannerPose(scanner_pose_v);
  ROS_DEBUG("Received planar scanner's pose wrt robot: %.3f %.3f %.3f",
            scanner_pose_v[0], scanner_pose_v[1], scanner_pose_v[2]);
}

void Node2D::initLatestScanData(const sensor_msgs::LaserScanConstPtr& planar_scan,
                                int scanner_index)
{
  latest_scan_data_ = std::make_shared<PlanarData>();
  latest_scan_data_->range_count_ = planar_scan->ranges.size();
}

bool Node2D::getAngleStats(const sensor_msgs::LaserScanConstPtr& planar_scan, double* angle_min, double* angle_increment)
{
  // To account for the planar scanners that are mounted upside-down, we determine the
  // min, max, and increment angles of the scanner in the base frame.
  //
  // Construct min and max angles of scanner, in the base_link frame.
  tf2::Quaternion min_q;
  min_q.setRPY(0.0, 0.0, planar_scan->angle_min);
  tf2::Quaternion inc_q;
  inc_q.setRPY(0.0, 0.0, planar_scan->angle_min + planar_scan->angle_increment);
  geometry_msgs::Quaternion min_q_msg = tf2::toMsg(min_q);
  geometry_msgs::Quaternion inc_q_msg = tf2::toMsg(inc_q);
  bool success = true;
  try
  {
    geometry_msgs::TransformStamped t = tf_buffer_.lookupTransform(node_->getBaseFrameId(), planar_scan->header.frame_id,
                                                                   planar_scan->header.stamp, ros::Duration(5.0));
    tf2::doTransform(min_q_msg, min_q_msg, t);
    tf2::doTransform(inc_q_msg, inc_q_msg, t);
  }
  catch (tf2::TransformException& e)
  {
    ROS_WARN_STREAM("Unable to transform min/max planar scanner angles into base frame: " << e.what());
    success = false;
  }
  if(success)
  {
    tf2::fromMsg(min_q_msg, min_q);
    tf2::fromMsg(inc_q_msg, inc_q);
    *angle_min = tf2::getYaw(min_q);
    *angle_increment = tf2::getYaw(inc_q) - *angle_min;
    // wrapping angle to [-pi .. pi]
    *angle_increment = angles::normalize_angle(*angle_increment);
  }
  return success;
}

void Node2D::updateLatestScanData(const sensor_msgs::LaserScanConstPtr& planar_scan,
                                  double angle_min, double angle_increment)
{
  // Apply range min/max thresholds, if the user supplied them
  if (sensor_max_range_ > 0.0)
    latest_scan_data_->range_max_ = std::min(planar_scan->range_max, static_cast<float>(sensor_max_range_));
  else
    latest_scan_data_->range_max_ = planar_scan->range_max;
  double range_min;
  if (sensor_min_range_ > 0.0)
    range_min = std::max(planar_scan->range_min, static_cast<float>(sensor_min_range_));
  else
    range_min = planar_scan->range_min;
  latest_scan_data_->ranges_.resize(latest_scan_data_->range_count_);
  latest_scan_data_->angles_.resize(latest_scan_data_->range_count_);
  for (int i = 0; i < latest_scan_data_->range_count_; i++)
  {
    // amcl doesn't (yet) have a concept of min range.  So we'll map short
    // readings to max range.
    if (planar_scan->ranges[i] <= range_min)
      latest_scan_data_->ranges_[i] = latest_scan_data_->range_max_;
    else
      latest_scan_data_->ranges_[i] = planar_scan->ranges[i];
    // Compute bearing
    latest_scan_data_->angles_[i] = angle_min + (i * angle_increment);
  }
}

void Node2D::resampleParticles()
{
  pf_->updateResample();
  if (pf_->isConverged() && global_localization_active_)
  {
    ROS_INFO("Global localization converged!");
    global_localization_active_ = false;
  }
}

bool Node2D::resamplePose(const ros::Time& stamp)
{
  double max_weight = 0.0;
  Eigen::Vector3d max_pose;
  getMaxWeightPose(&max_weight, &max_pose);
  bool success = true;
  if(max_weight > 0.0)
    success = updatePose(max_pose, stamp);
  else
  {
    ROS_ERROR("No pose!");
    success = false;
  }
  return success;
}

void Node2D::getMaxWeightPose(double* max_weight_rtn, Eigen::Vector3d* max_pose)
{
  // Read out the current hypotheses
  double max_weight = 0.0;
  int max_weight_hyp = -1;
  int cluster_count = pf_->getCurrentSet()->cluster_count;
  std::vector<PoseHypothesis> hyps;
  hyps.resize(cluster_count);
  for (int hyp_count = 0; hyp_count < cluster_count; hyp_count++)
  {
    double weight;
    Eigen::Vector3d pose_mean;
    if (!pf_->getClusterStats(hyp_count, &weight, &pose_mean))
    {
      ROS_ERROR_STREAM("Couldn't get stats on cluster " << hyp_count);
      break;
    }

    hyps[hyp_count].weight = weight;
    hyps[hyp_count].mean = pose_mean;

    if (hyps[hyp_count].weight > max_weight)
    {
      max_weight = hyps[hyp_count].weight;
      max_weight_hyp = hyp_count;
    }
  }
  *max_weight_rtn = max_weight;
  *max_pose = hyps[max_weight_hyp].mean;
}

bool Node2D::updatePose(const Eigen::Vector3d& max_pose, const ros::Time& stamp)
{
  ROS_DEBUG("Max weight pose: %.3f %.3f %.3f", max_pose[0], max_pose[1], max_pose[2]);
  node_->updatePose(max_pose, stamp);
  bool success = true;
  // subtracting base to odom from map to base and send map to odom instead
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, max_pose[2]);
  tf2::Transform base_to_map_tf(q, tf2::Vector3(max_pose[0], max_pose[1], 0.0));
  base_to_map_tf = base_to_map_tf.inverse();
  std::string odom_frame_id = node_->getOdomFrameId();
  std::string base_frame_id = node_->getBaseFrameId();
  geometry_msgs::Pose base_to_map_msg, odom_to_map_msg;
  base_to_map_msg.position = tf2::toMsg(base_to_map_tf.getOrigin(), base_to_map_msg.position);
  base_to_map_msg.orientation = tf2::toMsg(base_to_map_tf.getRotation());
  try
  {
    geometry_msgs::TransformStamped t;
    t = tf_buffer_.lookupTransform(odom_frame_id, base_frame_id, stamp, ros::Duration(1.0));
    tf2::doTransform(base_to_map_msg, odom_to_map_msg, t);
  }
  catch (tf2::TransformException)
  {
    ROS_DEBUG("Failed to subtract base to odom transform");
    success = false;
  }

  if(success)
  {
    tf2::Transform odom_to_map_transform;
    tf2::fromMsg(odom_to_map_msg, odom_to_map_transform);
    node_->updateOdomToMapTransform(odom_to_map_transform);
  }
  return success;
}

void Node2D::checkScanReceived(const ros::TimerEvent& event)
{
  ros::Duration d = ros::Time::now() - latest_scan_received_ts_;
  if (d > check_scanner_interval_)
  {
    ROS_WARN_STREAM("No planar scan received (and thus no pose updates have been published) for " << d
                    << " seconds. Verify that data is being published to the topic " << scan_sub_->getTopic() << ".");
  }
}

void Node2D::globalLocalizationCallback()
{
  scanner_.setMapFactors(global_localization_off_map_factor_,
                         global_localization_non_free_space_factor_,
                         non_free_space_radius_);
  for (auto& l : scanners_)
  {
    l->setMapFactors(global_localization_off_map_factor_,
                     global_localization_non_free_space_factor_,
                     non_free_space_radius_);
  }
}

}  // namespace amcl

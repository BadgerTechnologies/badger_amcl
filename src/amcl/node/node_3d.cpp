/*
 *
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

#include "node/node_3d.h"

#include <functional>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose2D.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/impl/point_types.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/projection_matrix.h>
#include <octomap/AbstractOcTree.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <ros/assert.h>
#include <ros/console.h>
#include <ros/names.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "node/node.h"

namespace badger_amcl
{

Node3D::Node3D(Node* node, std::mutex& configuration_mutex)
    : node_(node),
      configuration_mutex_(configuration_mutex),
      private_nh_("~"),
      resample_count_(0),
      tf_listener_(tf_buffer_)
{
  map_ = nullptr;
  octree_ = nullptr;
  latest_scan_data_ = NULL;
  fake_sample_set_ = std::make_shared<PFSampleSet>();
  private_nh_.param("first_map_only", first_map_only_, false);
  private_nh_.param("wait_for_occupancy_map", wait_for_occupancy_map_, false);
  private_nh_.param("laser_max_beams", max_beams_, 256);
  private_nh_.param("laser_z_hit", z_hit_, 0.95);
  private_nh_.param("laser_z_rand", z_rand_, 0.05);
  private_nh_.param("laser_sigma_hit", sigma_hit_, 0.2);
  private_nh_.param("laser_off_map_factor", off_map_factor_, 1.0);
  private_nh_.param("laser_non_free_space_factor", non_free_space_factor_, 1.0);
  private_nh_.param("laser_non_free_space_radius", non_free_space_radius_, 0.0);
  private_nh_.param("laser_likelihood_max_dist", max_distance_to_object_, 0.36);
  private_nh_.param("resample_interval", resample_interval_, 2);
  private_nh_.param("laser_gompertz_a", gompertz_a_, 1.0);
  private_nh_.param("laser_gompertz_b", gompertz_b_, 1.0);
  private_nh_.param("laser_gompertz_c", gompertz_c_, 1.0);
  private_nh_.param("laser_gompertz_input_shift", gompertz_input_shift_, 0.0);
  private_nh_.param("laser_gompertz_input_scale", gompertz_input_scale_, 1.0);
  private_nh_.param("laser_gompertz_output_shift", gompertz_output_shift_, 0.0);
  private_nh_.param("global_localization_scanner_off_map_factor", global_localization_off_map_factor_, 1.0);
  private_nh_.param("global_localization_scanner_non_free_space_factor",
                    global_localization_non_free_space_factor_, 1.0);
  std::string model_type_str;
  private_nh_.param("laser_model_type", model_type_str, std::string("likelihood_field_gompertz"));
  if (model_type_str == "likelihood_field")
  {
    model_type_ = POINT_CLOUD_MODEL;
  }
  else if (model_type_str == "likelihood_field_gompertz")
  {
    model_type_ = POINT_CLOUD_MODEL_GOMPERTZ;
  }
  else
  {
    ROS_WARN_STREAM("Unknown point cloud scanner model type \"" << model_type_str
                    << "\"; defaulting to point cloud scanner model");
    model_type_ = POINT_CLOUD_MODEL;
  }
  private_nh_.param("map_scale_up_factor", occupancy_map_scale_up_factor_, 1);

  cloud_topic_ = "cloud";
  cloud_sub_ = std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>>(
      new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, cloud_topic_, 1));
  cloud_filter_ = std::unique_ptr<tf2_ros::MessageFilter<sensor_msgs::PointCloud2>>(
      new tf2_ros::MessageFilter<sensor_msgs::PointCloud2>(*cloud_sub_, tf_buffer_, node_->getOdomFrameId(), 1, nh_));
  cloud_filter_->registerCallback(std::bind(&Node3D::scanReceived, this, std::placeholders::_1));
  // 15s timer to warn on lack of receipt of point cloud scans, #5209
  scanner_check_interval_ = ros::Duration(15.0);
  check_scanner_timer_ = nh_.createTimer(scanner_check_interval_, std::bind(&Node3D::checkScanReceived, this,
                                                                            std::placeholders::_1));
  force_update_ = false;
  first_occupancy_map_received_ = false;
  first_octomap_received_ = false;
  occupancy_bounds_received_ = false;
  octo_map_sub_ = nh_.subscribe("octomap", 1, &Node3D::octoMapMsgReceived, this);
  occupancy_map_sub_ = nh_.subscribe("map", 1, &Node3D::occupancyMapMsgReceived, this);
}

Node3D::~Node3D()
{
  // TF message filters must be destroyed before the underlying subsriber.
  cloud_filter_.reset();
}

void Node3D::reconfigure(AMCLConfig& config)
{
  resample_interval_ = config.resample_interval;
  max_beams_ = config.laser_max_beams;
  z_hit_ = config.laser_z_hit;
  z_short_ = config.laser_z_short;
  z_max_ = config.laser_z_max;
  z_rand_ = config.laser_z_rand;
  sigma_hit_ = config.laser_sigma_hit;
  max_distance_to_object_ = config.laser_likelihood_max_dist;
  off_map_factor_ = config.laser_off_map_factor;
  non_free_space_factor_ = config.laser_non_free_space_factor;
  non_free_space_radius_ = config.laser_non_free_space_radius;
  global_localization_off_map_factor_ = config.global_localization_laser_off_map_factor;
  global_localization_non_free_space_factor_ = config.global_localization_laser_non_free_space_factor;
  if (config.laser_model_type == "likelihood_field")
  {
    model_type_ = POINT_CLOUD_MODEL;
  }
  else if (config.laser_model_type == "likelihood_field_gompertz")
  {
    model_type_ = POINT_CLOUD_MODEL_GOMPERTZ;
  }
  scanner_.init(max_beams_, map_);
  if (model_type_ == POINT_CLOUD_MODEL)
  {
    ROS_WARN("Setting point cloud model type from reconfigure 3d");
    scanner_.setPointCloudModel(z_hit_, z_rand_, sigma_hit_);
  }
  else if (model_type_ == POINT_CLOUD_MODEL_GOMPERTZ)
  {
    ROS_INFO("Initializing likelihood field (gompertz) model; this can take some time on large maps...");
    scanner_.setPointCloudModelGompertz(
        z_hit_, z_rand_, sigma_hit_, gompertz_a_, gompertz_b_, gompertz_c_,
        gompertz_input_shift_, gompertz_input_scale_, gompertz_output_shift_);
    ROS_INFO("Gompertz key points by total planar scan match: "
             "0.0: %f, 0.25: %f, 0.5: %f, 0.75: %f, 1.0: %f",
             scanner_.applyGompertz(z_rand_),
             scanner_.applyGompertz(z_rand_ + z_hit_ * .25),
             scanner_.applyGompertz(z_rand_ + z_hit_ * .5),
             scanner_.applyGompertz(z_rand_ + z_hit_ * .75),
             scanner_.applyGompertz(z_rand_ + z_hit_));
    ROS_INFO("Done initializing likelihood (gompertz) field model.");
  }

  scanner_.setMapFactors(off_map_factor_, non_free_space_factor_, non_free_space_radius_);
  cloud_sub_ = std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>>(
      new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, cloud_topic_, 1));

  cloud_filter_ = std::unique_ptr<tf2_ros::MessageFilter<sensor_msgs::PointCloud2>>(
      new tf2_ros::MessageFilter<sensor_msgs::PointCloud2>(*cloud_sub_, tf_buffer_, node_->getOdomFrameId(), 1, nh_));
  cloud_filter_->registerCallback(std::bind(&Node3D::scanReceived, this, std::placeholders::_1));
  pf_ = node_->getPfPtr();
}

void Node3D::occupancyMapMsgReceived(const nav_msgs::OccupancyGridConstPtr& msg)
{
  std::lock_guard<std::mutex> cfl(configuration_mutex_);
  if(not wait_for_occupancy_map_ or (first_map_only_ && first_occupancy_map_received_))
    return;

  first_occupancy_map_received_ = true;
  std::vector<int> size_vec;
  double resolution = (*msg).info.resolution / occupancy_map_scale_up_factor_;
  size_vec.push_back((*msg).info.width * occupancy_map_scale_up_factor_);
  size_vec.push_back((*msg).info.height * occupancy_map_scale_up_factor_);
  occupancy_map_min_ = {0.0, 0.0};
  occupancy_map_max_ = {size_vec[0] * resolution, size_vec[1] * resolution};
  occupancy_bounds_received_ = true;
  if(first_octomap_received_)
  {
    map_->setMapBounds(occupancy_map_min_, occupancy_map_max_);
    updateFreeSpaceIndices();
  }
}

void Node3D::octoMapMsgReceived(const octomap_msgs::OctomapConstPtr& msg)
{
  if (first_map_only_ && first_octomap_received_)
  {
    ROS_DEBUG("Octomap already received");
    return;
  }

  std::lock_guard<std::mutex> cfl(configuration_mutex_);
  ROS_INFO("Received a new Octomap");
  map_ = convertMap(*msg);

  // Clear queued point cloud objects because they hold pointers to the existing map
  scanners_.clear();
  scanners_update_.clear();
  frame_to_scanner_.clear();
  latest_scan_data_ = NULL;
  initFromNewMap();
  first_octomap_received_ = true;
}

void Node3D::initFromNewMap()
{
  scanner_.init(max_beams_, map_);
  if (model_type_ == POINT_CLOUD_MODEL)
  {
    scanner_.setPointCloudModel(z_hit_, z_rand_, sigma_hit_);
  }
  else if (model_type_ == POINT_CLOUD_MODEL_GOMPERTZ)
  {
    ROS_INFO("Initializing likelihood field (gompertz) model; this can take some time on large maps...");
    scanner_.setPointCloudModelGompertz(z_hit_, z_rand_, sigma_hit_, gompertz_a_, gompertz_b_, gompertz_c_,
                                        gompertz_input_shift_, gompertz_input_scale_, gompertz_output_shift_);
    ROS_INFO("Gompertz key points by total planar scan match: "
             "0.0: %f, 0.25: %f, 0.5: %f, 0.75: %f, 1.0: %f",
             scanner_.applyGompertz(z_rand_),
             scanner_.applyGompertz(z_rand_ + z_hit_ * .25),
             scanner_.applyGompertz(z_rand_ + z_hit_ * .5),
             scanner_.applyGompertz(z_rand_ + z_hit_ * .75),
             scanner_.applyGompertz(z_rand_ + z_hit_));
    ROS_INFO("Done initializing likelihood (gompertz) field model.");
  }
  scanner_.setMapFactors(off_map_factor_, non_free_space_factor_, non_free_space_radius_);
  node_->initFromNewMap(map_, not first_octomap_received_);
  pf_ = node_->getPfPtr();
  // if we are using both maps as bounds
  // and the occupancy map has already arrived
  if (wait_for_occupancy_map_ and occupancy_bounds_received_)
  {
    map_->setMapBounds(occupancy_map_min_, occupancy_map_max_);
    updateFreeSpaceIndices();
  }
  else if(!wait_for_occupancy_map_)
  {
    map_->updateDistancesLUT();
    updateFreeSpaceIndices();
  }
}

/**
 * Convert a octomap message into the internal
 * representation.  This allocates an OctoMap and returns it.
 */
std::shared_ptr<OctoMap> Node3D::convertMap(const octomap_msgs::Octomap& map_msg)
{
  octomap::AbstractOcTree* absoctree = nullptr;
  bool binary = map_msg.binary;
  if (binary)
  {
    absoctree = octomap_msgs::binaryMsgToMap(map_msg);
  }
  else
  {
    absoctree = octomap_msgs::fullMsgToMap(map_msg);
  }
  if (absoctree)
  {
    octree_ = std::shared_ptr<octomap::OcTree>(dynamic_cast<octomap::OcTree*>(absoctree));
  }
  double resolution = map_msg.resolution;
  std::shared_ptr<OctoMap> octomap = std::make_shared<OctoMap>(resolution);
  ROS_ASSERT(octomap);
  octomap->initFromOctree(octree_, max_distance_to_object_);
  octree_.reset();
  return octomap;
}

double Node3D::scorePose(const Eigen::Vector3d& p)
{
  // If there is no data to match, return a perfect match
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

void Node3D::updateFreeSpaceIndices()
{
  // TODO: update free space indices with initialized 2D map
  // Index of free space
  // Must be calculated after the distances lut is set
  std::vector<std::pair<int, int>> fsi;
  std::vector<int> min_cells(3), max_cells(3);
  map_->getMinMaxCells(&min_cells, &max_cells);
  for (int i = min_cells[0]; i < max_cells[0]; i++)
    for (int j = min_cells[1]; j < max_cells[1]; j++)
      fsi.push_back(std::make_pair(i, j));
  node_->updateFreeSpaceIndices(fsi);
}

void Node3D::scanReceived(const sensor_msgs::PointCloud2ConstPtr& point_cloud_scan)
{
  latest_scan_received_ts_ = ros::Time::now();
  if(!isMapInitialized())
    return;

  if (!global_localization_active_)
    deactivateGlobalLocalizationParams();

  ros::Time stamp = point_cloud_scan->header.stamp;
  int scanner_index = getFrameToScannerIndex(point_cloud_scan->header.frame_id);
  if(scanner_index >= 0)
  {
    bool force_publication = false, resampled = false, success;
    success = updateNodePf(stamp, scanner_index, &force_publication);
    if(scanners_update_.at(scanner_index))
      updateScanner(point_cloud_scan, scanner_index, &resampled);
    if(force_publication or resampled)
      success = success and resamplePose(stamp);
    if(success)
    {
      node_->attemptSavePose(false);
    }
  }
}

bool Node3D::updateNodePf(const ros::Time& stamp, int scanner_index, bool* force_publication)
{
  return node_->updatePf(stamp, scanners_update_, scanner_index, &resample_count_,
                         force_publication, &force_update_);
}

void Node3D::updateScanner(const sensor_msgs::PointCloud2ConstPtr& point_cloud_scan,
                           int scanner_index, bool* resampled)
{
  initLatestScanData(point_cloud_scan, scanner_index);
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  makePointCloudFromScan(point_cloud_scan, point_cloud);
  updateLatestScanData(point_cloud, scanner_index);
  scanners_[scanner_index]->updateSensor(pf_, std::dynamic_pointer_cast<SensorData>(
                                                latest_scan_data_));
  scanners_update_.at(scanner_index) = false;
  if(!(++resample_count_ % resample_interval_))
  {
    resampleParticles();
    *resampled = true;
  }
  if(!force_update_)
    node_->publishParticleCloud();
}

bool Node3D::isMapInitialized()
{
  if (map_ == NULL)
  {
    ROS_DEBUG("Map is null");
    return false;
  }
  if (not map_->isDistancesLUTCreated())
  {
    ROS_DEBUG("Distances not yet created");
    return false;
  }
  return true;
}

void Node3D::deactivateGlobalLocalizationParams()
{
  std::lock_guard<std::mutex> cfl(configuration_mutex_);
  // Handle corner cases like getting dynamically reconfigured or getting a
  // new map by de-activating the global localization parameters here.
  node_->setPfDecayRateNormal();
  scanner_.setMapFactors(off_map_factor_, non_free_space_factor_, non_free_space_radius_);
  for (auto& l : scanners_)
  {
    l->setMapFactors(off_map_factor_, non_free_space_factor_, non_free_space_radius_);
  }
}

int Node3D::getFrameToScannerIndex(const std::string& scanner_frame_id)
{
  int scanner_index;
  // Do we have the base->base_lidar Tx yet?
  if (frame_to_scanner_.find(scanner_frame_id) == frame_to_scanner_.end())
  {
    scanner_index = initFrameToScanner();
    if(scanner_index >= 0)
    {
      geometry_msgs::Transform scanner_to_footprint_tf;
      if(getFootprintToFrameTransform(scanner_frame_id, &scanner_to_footprint_tf))
      {
        frame_to_scanner_[scanner_frame_id] = scanner_index;
        scanners_[scanner_index]->setPointCloudScannerToFootprintTF(scanner_to_footprint_tf);
      }
      else
      {
        scanner_index = -1;
      }
    }
  }
  else
  {
    // we have the point cloud scanner pose, retrieve scanner index
    scanner_index = frame_to_scanner_[scanner_frame_id];
  }
  return scanner_index;
}

bool Node3D::getFootprintToFrameTransform(const std::string& scanner_frame_id, geometry_msgs::Transform* tf)
{
  std::string footprint_frame_id = node_->getBaseFrameId();
  try
  {
    *tf = tf_buffer_.lookupTransform(footprint_frame_id, scanner_frame_id,
                                     ros::Time::now(), ros::Duration(5.0)).transform;
  }
  catch (tf2::TransformException& e)
  {
    ROS_ERROR("Failed to get transform from base footprint to given frame.");
    return false;
  }
  return true;
}

int Node3D::initFrameToScanner()
{
  scanners_.push_back(std::make_shared<PointCloudScanner>(scanner_));
  scanners_update_.push_back(true);
  int scanner_index = frame_to_scanner_.size();
  return scanner_index;
}

void Node3D::initLatestScanData(const sensor_msgs::PointCloud2ConstPtr& point_cloud_scan, int scanner_index)
{
  latest_scan_data_ = std::make_shared<PointCloudData>();
  latest_scan_data_->frame_id_ = point_cloud_scan->header.frame_id;
}

void Node3D::makePointCloudFromScan(const sensor_msgs::PointCloud2ConstPtr& point_cloud_scan,
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud)
{
  pcl::PCLPointCloud2 pc2;
  pcl_conversions::toPCL(*point_cloud_scan, pc2);
  pcl::fromPCLPointCloud2(pc2, *point_cloud);
}

void Node3D::updateLatestScanData(const pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud, int scanner_index)
{
  int max_beams = scanners_[scanner_index]->getMaxBeams();
  int data_count = point_cloud->size();
  int step = (data_count - 1) / (max_beams - 1);
  step = std::max(step, 1);
  // Sample point cloud scan to max_beams number of points
  for (int i = 0; i < data_count; i += step)
  {
    pcl::PointXYZ point = point_cloud->at(i);
    latest_scan_data_->points_.push_back(point);
  }
  latest_scan_data_->points_.header = point_cloud->header;
}

void Node3D::resampleParticles()
{
  pf_->updateResample();
  if (pf_->isConverged() && global_localization_active_)
  {
    ROS_INFO("Global localization converged!");
    global_localization_active_ = false;
  }
}

bool Node3D::resamplePose(const ros::Time& stamp)
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

void Node3D::getMaxWeightPose(double* max_weight_rtn, Eigen::Vector3d* max_pose)
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

bool Node3D::updatePose(const Eigen::Vector3d& max_pose, const ros::Time& stamp)
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

void Node3D::checkScanReceived(const ros::TimerEvent& event)
{
  ros::Duration d = ros::Time::now() - latest_scan_received_ts_;
  if (d > scanner_check_interval_)
  {
    ROS_DEBUG_STREAM("No point cloud scan received (and thus no pose updates have been published) for " << d
                     << " seconds. Verify that data is being published on the topic " << cloud_sub_->getTopic() << ".");
  }
}

void Node3D::globalLocalizationCallback()
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

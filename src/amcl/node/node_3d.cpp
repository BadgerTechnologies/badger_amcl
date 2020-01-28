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
/*************************************************************************
 * Desc: AMCL Node for 3D AMCL
 * Author: Tyler Buchman (tyler_buchman@jabil.com)
 *************************************************************************/

#include "node/node_3d.h"

#include <boost/bind.hpp>
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
#include <tf/exceptions.h>
#include <tf/transform_datatypes.h>

#include "node/node.h"

using namespace amcl;

Node3D::Node3D(Node* node, int map_type, std::mutex& configuration_mutex)
    : node_(node)
    , map_type_(map_type)
    , configuration_mutex_(configuration_mutex)
    , private_nh_("~")
    , resample_count_(0)
{
  map_ = nullptr;
  octree_ = nullptr;
  latest_scan_data_ = NULL;
  fake_sample_set_ = std::make_shared<PFSampleSet>();
  private_nh_.param("first_map_only", first_map_only_, false);
  private_nh_.param("odom_frame_id", odom_frame_id_, std::string("odom"));
  private_nh_.param("base_frame_id", base_frame_id_, std::string("base_link"));
  private_nh_.param("global_frame_id", global_frame_id_, std::string("map"));
  private_nh_.param("global_alt_frame_id", global_alt_frame_id_, std::string(""));
  private_nh_.param("wait_for_occupancy_map", wait_for_occupancy_map_, false);
  private_nh_.param("point_cloud_scanner_max_beams", max_beams_, 256);
  private_nh_.param("point_cloud_scanner_z_hit", z_hit_, 0.95);
  private_nh_.param("point_cloud_scanner_z_rand", z_rand_, 0.05);
  private_nh_.param("point_cloud_scanner_sigma_hit", sigma_hit_, 0.2);
  private_nh_.param("point_cloud_scanner_off_map_factor", off_map_factor_, 1.0);
  private_nh_.param("point_cloud_scanner_non_free_space_factor", non_free_space_factor_, 1.0);
  private_nh_.param("point_cloud_scanner_non_free_space_radius", non_free_space_radius_, 0.0);
  private_nh_.param("point_cloud_scanner_likelihood_max_dist", sensor_likelihood_max_dist_, 0.36);
  private_nh_.param("resample_interval", resample_interval_, 2);
  private_nh_.param("point_cloud_gompertz_a", gompertz_a_, 1.0);
  private_nh_.param("point_cloud_gompertz_b", gompertz_b_, 1.0);
  private_nh_.param("point_cloud_gompertz_c", gompertz_c_, 1.0);
  private_nh_.param("point_cloud_gompertz_input_shift", gompertz_input_shift_, 0.0);
  private_nh_.param("point_cloud_gompertz_input_scale", gompertz_input_scale_, 1.0);
  private_nh_.param("point_cloud_gompertz_output_shift", gompertz_output_shift_, 0.0);
  private_nh_.param("global_localization_scanner_off_map_factor", global_localization_off_map_factor_, 1.0);
  private_nh_.param("global_localization_scanner_non_free_space_factor",
                    global_localization_non_free_space_factor_, 1.0);
  private_nh_.param("point_cloud_scanner_height", scanner_height_, 1.8);
  const std::string default_point_cloud_scan_topic = "/scans/top/points_filtered";
  private_nh_.param("point_cloud_scan_topic", scan_topic_, default_point_cloud_scan_topic);
  std::string tmp_model_type;
  private_nh_.param("point_cloud_model_type", tmp_model_type, std::string("point cloud"));
  if (tmp_model_type == "point cloud")
  {
    model_type_ = POINT_CLOUD_MODEL;
  }
  else if (tmp_model_type == "point cloud gompertz")
  {
    model_type_ = POINT_CLOUD_MODEL_GOMPERTZ;
  }
  else
  {
    ROS_WARN("Unknown point cloud scanner model type \"%s\"; defaulting to point cloud scanner model",
             tmp_model_type.c_str());
    model_type_ = POINT_CLOUD_MODEL;
  }

  if (map_type_ == 3)
  {
    scan_sub_ = std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>>(
        new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, scan_topic_, 1));
    scan_filter_ = std::unique_ptr<tf::MessageFilter<sensor_msgs::PointCloud2>>(
        new tf::MessageFilter<sensor_msgs::PointCloud2>(*scan_sub_, tf_, odom_frame_id_, 1));
    scan_filter_->registerCallback(boost::bind(&Node3D::scanReceived, this, _1));
    // 15s timer to warn on lack of receipt of point cloud scans, #5209
    scanner_check_interval_ = ros::Duration(15.0);
    check_scanner_timer_ =
        nh_.createTimer(scanner_check_interval_, boost::bind(&Node3D::checkScanReceived, this, _1));
  }

  try
  {
    tf_.waitForTransform("base_footprint", "top_laser", ros::Time::now(), ros::Duration(5.0));
    tf_.lookupTransform("base_footprint", "top_laser", ros::Time::now(), scanner_to_footprint_tf_);
  }
  catch (tf::TransformException& e)
  {
    ROS_ERROR("failed to get top laser to base footprint transform.");
    return;
  }

  scanners_update_ = std::make_shared<std::vector<bool>>();
  force_update_ = false;
  first_octomap_received_ = false;
  map_sub_ = nh_.subscribe("octomap_binary", 1, &Node3D::mapMsgReceived, this);
}

void Node3D::reconfigure(amcl::AMCLConfig& config)
{
  scan_topic_ = config.point_cloud_scan_topic;
  odom_frame_id_ = config.odom_frame_id;
  base_frame_id_ = config.base_frame_id;
  global_frame_id_ = config.global_frame_id;
  resample_interval_ = config.resample_interval;
  max_beams_ = config.point_cloud_scanner_max_beams;
  z_hit_ = config.point_cloud_scanner_z_hit;
  z_short_ = config.point_cloud_scanner_z_short;
  z_max_ = config.point_cloud_scanner_z_max;
  z_rand_ = config.point_cloud_scanner_z_rand;
  sigma_hit_ = config.point_cloud_scanner_sigma_hit;
  sensor_likelihood_max_dist_ = config.point_cloud_scanner_likelihood_max_dist;
  off_map_factor_ = config.point_cloud_scanner_off_map_factor;
  non_free_space_factor_ = config.point_cloud_scanner_non_free_space_factor;
  non_free_space_radius_ = config.point_cloud_scanner_non_free_space_radius;
  global_localization_off_map_factor_ = config.global_localization_point_cloud_scanner_off_map_factor;
  global_localization_non_free_space_factor_ = config.global_localization_point_cloud_scanner_non_free_space_factor;
  scanner_height_ = config.point_cloud_scanner_height;
  if (config.point_cloud_model_type == "point cloud")
  {
    model_type_ = POINT_CLOUD_MODEL;
  }
  else if (config.point_cloud_model_type == "point cloud gompertz")
  {
    model_type_ = POINT_CLOUD_MODEL_GOMPERTZ;
  }
  scanner_.init(max_beams_, map_, scanner_height_);
  if (model_type_ == POINT_CLOUD_MODEL)
  {
    ROS_WARN("setting point cloud model type from reconfigure 3d");
    scanner_.setPointCloudModel(z_hit_, z_rand_, sigma_hit_, sensor_likelihood_max_dist_);
    map_->updateCSpace();
  }
  else if (model_type_ == POINT_CLOUD_MODEL_GOMPERTZ)
  {
    ROS_INFO("Initializing likelihood field (gompertz) model; this can take some time on large maps...");
    scanner_.setPointCloudModelGompertz(
        z_hit_, z_rand_, sigma_hit_, sensor_likelihood_max_dist_, gompertz_a_, gompertz_b_,
        gompertz_c_, gompertz_input_shift_, gompertz_input_scale_,
        gompertz_output_shift_);
    ROS_INFO("Gompertz key points by total planar scan match: "
             "0.0: %f, 0.25: %f, 0.5: %f, 0.75: %f, 1.0: %f",
             scanner_.applyGompertz(z_rand_), scanner_.applyGompertz(z_rand_ + z_hit_ * .25),
             scanner_.applyGompertz(z_rand_ + z_hit_ * .5),
             scanner_.applyGompertz(z_rand_ + z_hit_ * .75),
             scanner_.applyGompertz(z_rand_ + z_hit_));
    ROS_INFO("Done initializing likelihood (gompertz) field model.");
  }

  scanner_.setMapFactors(off_map_factor_, non_free_space_factor_, non_free_space_radius_);
  scan_filter_ = std::unique_ptr<tf::MessageFilter<sensor_msgs::PointCloud2>>(
      new tf::MessageFilter<sensor_msgs::PointCloud2>(*scan_sub_, tf_, odom_frame_id_, 100));
  scan_filter_->registerCallback(boost::bind(&Node3D::scanReceived, this, _1));
}

void Node3D::mapMsgReceived(const octomap_msgs::OctomapConstPtr& msg)
{
  if (first_map_only_ && first_octomap_received_)
  {
    return;
  }
  ROS_INFO("Received a new Octomap");
  std::lock_guard<std::mutex> cfl(configuration_mutex_);

  map_ = convertMap(*msg);
  first_octomap_received_ = true;

  // if the OctoMap is a secondary map source
  if (map_type_ != 3)
    return;

  // Clear queued point cloud objects because they hold pointers to the existing map
  scanners_.clear();
  scanners_update_->clear();
  frame_to_scanner_.clear();
  latest_scan_data_ = NULL;
  initFromNewMap();
}

void Node3D::initFromNewMap()
{
  scanner_.init(max_beams_, map_, scanner_height_);
  if (model_type_ == POINT_CLOUD_MODEL)
  {
    scanner_.setPointCloudModel(z_hit_, z_rand_, sigma_hit_, sensor_likelihood_max_dist_);
  }
  else if (model_type_ == POINT_CLOUD_MODEL_GOMPERTZ)
  {
    ROS_INFO("Initializing likelihood field (gompertz) model; this can take some time on large maps...");
    scanner_.setPointCloudModelGompertz(
        z_hit_, z_rand_, sigma_hit_, sensor_likelihood_max_dist_, gompertz_a_, gompertz_b_,
        gompertz_c_, gompertz_input_shift_, gompertz_input_scale_, gompertz_output_shift_);
    ROS_INFO("Gompertz key points by total planar scan match: "
             "0.0: %f, 0.25: %f, 0.5: %f, 0.75: %f, 1.0: %f",
             scanner_.applyGompertz(z_rand_), scanner_.applyGompertz(z_rand_ + z_hit_ * .25),
             scanner_.applyGompertz(z_rand_ + z_hit_ * .5),
             scanner_.applyGompertz(z_rand_ + z_hit_ * .75),
             scanner_.applyGompertz(z_rand_ + z_hit_));
    ROS_INFO("Done initializing likelihood (gompertz) field model.");
  }
  scanner_.setMapFactors(off_map_factor_, non_free_space_factor_, non_free_space_radius_);
  node_->initFromNewMap(map_);
  // if we are using both maps as bounds
  // and the occupancy map has already arrived
  if (wait_for_occupancy_map_ and occupancy_bounds_received_)
  {
    map_->setMapBounds(occupancy_map_min_, occupancy_map_max_);
    updateFreeSpaceIndices();
  }
  else if(!wait_for_occupancy_map_)
  {
    map_->updateCSpace();
    updateFreeSpaceIndices();
  }
}

/**
 * Convert a octomap message into the internal
 * representation.  This allocates an OctoMap and returns it.
 */
std::shared_ptr<OctoMap> Node3D::convertMap(const octomap_msgs::Octomap& map_msg)
{
  std::shared_ptr<OctoMap> octomap = std::make_shared<OctoMap>(wait_for_occupancy_map_);
  ROS_ASSERT(octomap);
  octomap::AbstractOcTree* absoctree;
  double resolution = map_msg.resolution;
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
  octomap->setResolution(resolution);
  octomap->initFromOctree(octree_, scanner_height_);
  return octomap;
}

double Node3D::scorePose(const PFVector& p)
{
  if (latest_scan_data_ == NULL)
  {
    // There is no data to match, so return a perfect match
    return 1.0;
  }
  // Create a fake "sample set" of just this pose to score it.
  fake_sample_.pose.v[0] = p.v[0];
  fake_sample_.pose.v[1] = p.v[1];
  fake_sample_.pose.v[2] = p.v[2];
  fake_sample_.weight = 1.0;
  fake_sample_set_->sample_count = 1;
  fake_sample_set_->samples = { fake_sample_ };
  fake_sample_set_->converged = 0;
  scanner_.applyModelToSampleSet(latest_scan_data_, fake_sample_set_);
  return fake_sample_.weight;
}

void Node3D::setOctomapBoundsFromOccupancyMap(std::shared_ptr<std::vector<double>> map_min,
                                              std::shared_ptr<std::vector<double>> map_max)
{
  occupancy_map_min_ = map_min;
  occupancy_map_max_ = map_max;
  occupancy_bounds_received_ = true;
  if(wait_for_occupancy_map_ and first_octomap_received_)
  {
    map_->setMapBounds(map_min, map_max);
    updateFreeSpaceIndices();
  }
}

void Node3D::updateFreeSpaceIndices()
{
  // Index of free space
  // Must be calculated after the occ_dist is setup by the laser model
  std::shared_ptr<std::vector<std::pair<int, int>>> fsi =
          std::make_shared<std::vector<std::pair<int, int>>>();
  fsi->resize(0);
  std::vector<int> min_cells(3), max_cells(3);
  map_->getMinMaxCells(&min_cells, &max_cells);
  for (int i = min_cells[0]; i < max_cells[0]; i++)
    for (int j = min_cells[1]; j < max_cells[1]; j++)
      fsi->push_back(std::make_pair(i, j));
  node_->updateFreeSpaceIndices(fsi);
}

void Node3D::scanReceived(const sensor_msgs::PointCloud2ConstPtr& point_cloud_scan)
{
  latest_scan_received_ts_ = ros::Time::now();
  if (map_ == NULL)
  {
    ROS_DEBUG("Map is null");
    return;
  }
  if (not map_->isCSpaceCreated())
  {
    ROS_DEBUG("CSpace not yet created");
    return;
  }

  std::lock_guard<std::mutex> cfl(configuration_mutex_);
  int scanner_index = -1;

  // Handle corner cases like getting dynamically reconfigured or getting a
  // new map by de-activating the global localization parameters here if we are
  // no longer globally localizing.
  if (!global_localization_active_)
  {
    node_->setPfDecayRateNormal();
    scanner_.setMapFactors(off_map_factor_, non_free_space_factor_, non_free_space_radius_);
    for (auto& l : scanners_)
    {
      l->setMapFactors(off_map_factor_, non_free_space_factor_, non_free_space_radius_);
    }
  }

  // Do we have the base->base_lidar Tx yet?
  if (frame_to_scanner_.find(point_cloud_scan->header.frame_id) == frame_to_scanner_.end())
  {
    if(!initFrameToScanner(point_cloud_scan, &scanner_index))
      return;
  }
  else
  {
    // we have the point cloud scanner pose, retrieve scanner index
    scanner_index = frame_to_scanner_[point_cloud_scan->header.frame_id];
  }

  bool force_publication = false;
  if(!node_->updatePf(point_cloud_scan->header.stamp, scanners_update_, scanner_index,
                      &resample_count_, &force_publication, &force_update_))
    return;
  pf_ = node_->getPfPtr();
  bool resampled = false;
  // If the robot has moved, update the filter
  if (scanners_update_->at(scanner_index))
  {
    updatePf(point_cloud_scan, scanner_index, &resampled);
  }

  if (resampled || force_publication)
  {
    if(!resamplePf(point_cloud_scan))
      return;
  }
  else
  {
    node_->attemptSavePose();
  }
}

bool Node3D::initFrameToScanner(const sensor_msgs::PointCloud2ConstPtr& point_cloud_scan, int* scanner_index)
{
  scanners_.push_back(std::make_shared<PointCloudScanner>(scanner_));
  scanners_update_->push_back(true);
  *scanner_index = frame_to_scanner_.size();

  tf::Stamped<tf::Pose> ident(tf::Transform(tf::createIdentityQuaternion(), tf::Vector3(0, 0, 0)), ros::Time(),
                              point_cloud_scan->header.frame_id);
  tf::Stamped<tf::Pose> scanner_pose;
  try
  {
    tf_.transformPose(base_frame_id_, ident, scanner_pose);
  }
  catch (tf::TransformException& e)
  {
    ROS_ERROR("Couldn't transform from %s to %s, "
              "even though the message notifier is in use",
              point_cloud_scan->header.frame_id.c_str(), base_frame_id_.c_str());
    return false;
  }

  PFVector scanner_pose_v;
  scanner_pose_v.v[0] = scanner_pose.getOrigin().x();
  scanner_pose_v.v[1] = scanner_pose.getOrigin().y();
  // point cloud scanner mounting angle gets computed later -> set to 0 here!
  scanner_pose_v.v[2] = 0;
  scanners_[*scanner_index]->setPointCloudScannerPose(scanner_pose_v);
  scanners_[*scanner_index]->setPointCloudScannerToFootprintTF(scanner_to_footprint_tf_);
  ROS_DEBUG("Received point cloud scanner's pose wrt robot: %.3f %.3f %.3f", scanner_pose_v.v[0],
            scanner_pose_v.v[1], scanner_pose_v.v[2]);

  frame_to_scanner_[point_cloud_scan->header.frame_id] = *scanner_index;
  return true;
}

void Node3D::updatePf(const sensor_msgs::PointCloud2ConstPtr& point_cloud_scan,
                      int scanner_index, bool* resampled)
{
  latest_scan_data_ = std::make_shared<PointCloudData>();
  latest_scan_data_->sensor_ = scanners_[scanner_index];
  latest_scan_data_->frame_id_ = point_cloud_scan->header.frame_id;
  pcl::PCLPointCloud2 pc2;
  pcl_conversions::toPCL(*point_cloud_scan, pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pc2, *point_cloud);

  // sample point cloud
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
  scanners_[scanner_index]->updateSensor(pf_, std::dynamic_pointer_cast<SensorData>(latest_scan_data_));
  scanners_update_->at(scanner_index) = false;

  // Resample the particles
  if (!(++resample_count_ % resample_interval_))
  {
    pf_->updateResample();
    *resampled = true;
    if (pf_->isConverged() && global_localization_active_)
    {
      ROS_INFO("Global localization converged!");
      global_localization_active_ = false;
    }
  }

  // TODO: set maximum rate for publishing
  if(!force_update_)
  {
    node_->publishPfCloud();
  }
}

bool Node3D::resamplePf(const sensor_msgs::PointCloud2ConstPtr& point_cloud_scan)
{
  // Read out the current hypotheses
  double max_weight = 0.0;
  int max_weight_hyp = -1;
  int cluster_count = pf_->getCurrentSet()->cluster_count;
  std::vector<PoseHypothesis> hyps;
  hyps.resize(cluster_count);
  double weight;
  PFVector pose_mean;
  PFMatrix pose_cov;
  for (int hyp_count = 0; hyp_count < cluster_count; hyp_count++)
  {
    if (!pf_->getClusterStats(hyp_count, &weight, &pose_mean, &pose_cov))
    {
      ROS_ERROR("Couldn't get stats on cluster %d", hyp_count);
      break;
    }
    hyps[hyp_count].weight = weight;
    hyps[hyp_count].mean = pose_mean;
    hyps[hyp_count].covariance = pose_cov;
    if (hyps[hyp_count].weight > max_weight)
    {
      max_weight = hyps[hyp_count].weight;
      max_weight_hyp = hyp_count;
    }
  }
  if (max_weight > 0.0)
  {
    geometry_msgs::PoseWithCovarianceStamped p;
    // Fill in the header
    p.header.frame_id = global_frame_id_;
    p.header.stamp = point_cloud_scan->header.stamp;
    // Copy in the pose
    p.pose.pose.position.x = hyps[max_weight_hyp].mean.v[0];
    p.pose.pose.position.y = hyps[max_weight_hyp].mean.v[1];
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(hyps[max_weight_hyp].mean.v[2]),
                          p.pose.pose.orientation);
    // Copy in the covariance, converting from 3-D to 6-D
    std::shared_ptr<PFSampleSet> set = pf_->getCurrentSet();
    for (int i = 0; i < 2; i++)
    {
      for (int j = 0; j < 2; j++)
      {
        // Report the overall filter covariance, rather than the
        // covariance for the highest-weight cluster
        p.pose.covariance[6 * i + j] = set->cov.m[i][j];
      }
    }
    // Report the overall filter covariance, rather than the
    // covariance for the highest-weight cluster
    p.pose.covariance[6 * 5 + 5] = set->cov.m[2][2];
    node_->publishPose(p);
    // subtracting base to odom from map to base and send map to odom instead
    tf::Stamped<tf::Pose> odom_to_map;
    try
    {
      tf::Transform tmp_tf(
          tf::createQuaternionFromYaw(hyps[max_weight_hyp].mean.v[2]),
          tf::Vector3(hyps[max_weight_hyp].mean.v[0], hyps[max_weight_hyp].mean.v[1], 0.0));
      tf::Stamped<tf::Pose> tmp_tf_stamped(tmp_tf.inverse(), point_cloud_scan->header.stamp, base_frame_id_);
      tf_.waitForTransform(base_frame_id_, odom_frame_id_, point_cloud_scan->header.stamp, ros::Duration(1.0));
      tf_.transformPose(odom_frame_id_, tmp_tf_stamped, odom_to_map);
    }
    catch (tf::TransformException e)
    {
      ROS_WARN("Failed to subtract base to odom transform: %s", e.what());
      return false;
    }
    if(!node_->updateTf(odom_to_map))
      return false;
  }
  else
  {
    ROS_ERROR("No pose!");
  }
  return true;
}

void Node3D::checkScanReceived(const ros::TimerEvent& event)
{
  ros::Duration d = ros::Time::now() - latest_scan_received_ts_;
  if (d > scanner_check_interval_)
  {
    ROS_DEBUG("No point cloud scan received (and thus no pose updates have been published) for %f seconds. "
              "Verify that data is being published on the %s topic.",
              d.toSec(), ros::names::resolve(scan_topic_).c_str());
  }
}

void Node3D::globalLocalizationCallback()
{
  scanner_.setMapFactors(global_localization_off_map_factor_, global_localization_non_free_space_factor_,
                                     non_free_space_radius_);
  for (auto& l : scanners_)
  {
    l->setMapFactors(global_localization_off_map_factor_, global_localization_non_free_space_factor_,
                     non_free_space_radius_);
  }
}

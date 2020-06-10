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

#ifndef AMCL_NODE_NODE_3D_H
#define AMCL_NODE_NODE_3D_H

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <message_filters/subscriber.h>
#include <nav_msgs/OccupancyGrid.h>
#include <octomap_msgs/Octomap.h>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <ros/time.h>
#include <ros/timer.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/message_filter.h>

#include "badger_amcl/AMCLConfig.h"
#include "map/octomap.h"
#include "node/node_nd.h"
#include "pf/pf_vector.h"
#include "sensors/point_cloud_scanner.h"

namespace badger_amcl
{

class Node;

class Node3D : public NodeND
{
public:
  // The configuration_mutex parameter is a reference to the configuration_mutex in the Node class.
  // While this couples the Node class with the NodeND class, it is acceptable because the
  // Node class is designed to be coupled with a Map/Sensor combination, and the NodeND classes
  // are designed to be coupled with the Node class.
  Node3D(Node* node, std::mutex& configuration_mutex);
  ~Node3D();
  void reconfigure(AMCLConfig& config) override;
  void globalLocalizationCallback() override;
  double scorePose(const PFVector& p) override;
private:
  void scanReceived(const sensor_msgs::PointCloud2ConstPtr& point_cloud_scan);
  bool updateNodePf(const ros::Time& stamp, int scanner_index, bool* force_publication);
  void occupancyMapMsgReceived(const nav_msgs::OccupancyGridConstPtr& msg);
  void octoMapMsgReceived(const octomap_msgs::OctomapConstPtr& msg);
  void initFromNewMap();
  std::shared_ptr<OctoMap> convertMap(const octomap_msgs::Octomap& map_msg);
  bool initFrameToScanner(const sensor_msgs::PointCloud2ConstPtr& point_cloud_scan, int* scanner_index);
  bool updatePf(const sensor_msgs::PointCloud2ConstPtr& point_cloud_scan, int scanner_index, bool* resampled);
  bool resamplePf(const sensor_msgs::PointCloud2ConstPtr& point_cloud_scan);
  void makePointCloudFromScan(const sensor_msgs::PointCloud2ConstPtr& point_cloud_scan,
                              pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud);
  void updateFreeSpaceIndices();
  void updateLatestScanData(const pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud, int scanner_index);
  void updateScanner(const sensor_msgs::PointCloud2ConstPtr& point_cloud_scan, int scanner_index, bool* resampled);
  void resampleParticles();
  bool resamplePose(const ros::Time& stamp);
  void getMaxWeightPose(double* max_weight, PFVector* max_pose);
  bool updatePose(const PFVector& max_hyp_mean, const ros::Time& stamp);
  bool isMapInitialized();
  void deactivateGlobalLocalizationParams();
  int getFrameToScannerIndex(const std::string& frame_id);
  bool getFootprintToFrameTransform(const std::string& frame_id, tf::StampedTransform* StampedTransform);
  int initFrameToScanner(const std::string& frame_id);
  void initLatestScanData(const sensor_msgs::PointCloud2ConstPtr& point_cloud_scan, int scanner_index);
  void checkScanReceived(const ros::TimerEvent& event);

  std::shared_ptr<OctoMap> map_;
  std::shared_ptr<octomap::OcTree> octree_;
  std::shared_ptr<PointCloudData> latest_scan_data_;
  std::shared_ptr<PFSampleSet> fake_sample_set_;
  std::shared_ptr<ParticleFilter> pf_;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> scan_sub_;
  std::unique_ptr<tf::MessageFilter<sensor_msgs::PointCloud2>> scan_filter_;
  std::string cloud_topic_;
  std::map<std::string, int> frame_to_scanner_;
  std::mutex& configuration_mutex_;
  std::vector<std::shared_ptr<PointCloudScanner> > scanners_;
  std::vector<double> occupancy_map_min_, occupancy_map_max_;
  std::vector<bool> scanners_update_;
  PFSample fake_sample_;
  PointCloudModelType model_type_;
  PointCloudScanner scanner_;
  Node* node_;
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber occupancy_map_sub_;
  ros::Subscriber octo_map_sub_;
  ros::Duration scanner_check_interval_;
  ros::Timer check_scanner_timer_;
  ros::Time latest_scan_received_ts_;
  tf::TransformListener tf_;
  int occupancy_map_scale_up_factor_;
  int max_beams_;
  int resample_interval_;
  int resample_count_;
  bool first_occupancy_map_received_;
  bool first_octomap_received_;
  bool occupancy_bounds_received_;
  bool first_map_only_;
  bool wait_for_occupancy_map_;
  bool force_update_;  // used to temporarily let amcl update samples even when no motion occurs...
  double scanner_height_;
  double gompertz_a_;
  double gompertz_b_;
  double gompertz_c_;
  double gompertz_input_shift_;
  double gompertz_input_scale_;
  double gompertz_output_shift_;
  double max_distance_to_object_;
  double off_map_factor_;
  double non_free_space_factor_;
  double non_free_space_radius_;
  double z_hit_, z_short_, z_max_, z_rand_, sigma_hit_, lambda_short_;
  double global_localization_off_map_factor_;
  double global_localization_non_free_space_factor_;
  bool global_localization_active_;
};

}  // namespace amcl

#endif // AMCL_NODE_NODE_3D_H

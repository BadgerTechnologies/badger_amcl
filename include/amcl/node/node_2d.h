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

#ifndef AMCL_NODE_NODE_2D_H
#define AMCL_NODE_NODE_2D_H

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <message_filters/subscriber.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/duration.h>
#include <ros/subscriber.h>
#include <ros/time.h>
#include <ros/timer.h>
#include <tf/message_filter.h>

#include "badger_amcl/AMCLConfig.h"
#include "map/occupancy_map.h"
#include "node/node_nd.h"
#include "sensors/planar_scanner.h"

namespace badger_amcl
{

class Node;

class Node2D : public NodeND
{
public:
  // The configuration_mutex parameter is a reference to the configuration_mutex in the Node class.
  // While this couples the Node class with the NodeND class, it is acceptable because the
  // Node class is designed to be coupled with a Map/Sensor combination, and the NodeND classes
  // are designed to be coupled with the Node class.
  Node2D(Node* node, std::mutex& configuration_mutex);
  ~Node2D();
  void reconfigure(AMCLConfig& config) override;
  void globalLocalizationCallback() override;
  double scorePose(const Eigen::Vector3d& p) override;
private:
  void scanReceived(const sensor_msgs::LaserScanConstPtr& planar_scan);
  bool updateNodePf(const ros::Time& stamp, int scanner_index, bool* force_publication);
  bool updateScanner(const sensor_msgs::LaserScanConstPtr& planar_scan, int scanner_index, bool* resampled);
  void updateFreeSpaceIndices();
  void resampleParticles();
  bool resamplePose(const ros::Time& stamp);
  void getMaxWeightPose(double* max_weight_rtn, Eigen::Vector3d* max_pose);
  bool updatePose(const Eigen::Vector3d& max_pose, const ros::Time& stamp);
  bool isMapInitialized();
  void deactivateGlobalLocalizationParams();
  int getFrameToScannerIndex(const std::string& frame_id);
  void mapMsgReceived(const nav_msgs::OccupancyGridConstPtr& msg);
  void initFromNewMap();
  std::shared_ptr<OccupancyMap> convertMap(const nav_msgs::OccupancyGrid& map_msg);
  void checkScanReceived(const ros::TimerEvent& event);
  bool initFrameToScanner(const std::string& frame_id, tf::Stamped<tf::Pose>* scanner_pose, int* scanner_index);
  void updateScannerPose(const tf::Stamped<tf::Pose>& scanner_pose, int scanner_index);
  bool initLatestScanData(const sensor_msgs::LaserScanConstPtr& planar_scan, int scanner_index);
  bool getAngleStats(const sensor_msgs::LaserScanConstPtr& planar_scan, double* angle_min, double* angle_increment);
  void updateLatestScanData(const sensor_msgs::LaserScanConstPtr& planar_scan, double angle_min, double angle_increment);
  bool updatePf(const sensor_msgs::LaserScanConstPtr& planar_scan, int scanner_index, bool* resampled);
  bool resamplePf(const sensor_msgs::LaserScanConstPtr& planar_scan);

  Node* node_;
  std::shared_ptr<OccupancyMap> map_;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>> scan_sub_;
  std::unique_ptr<tf::MessageFilter<sensor_msgs::LaserScan>> scan_filter_;
  std::string scan_topic_;
  std::map<std::string, int> frame_to_scanner_;
  std::mutex& configuration_mutex_;
  std::vector<std::shared_ptr<PlanarScanner>> scanners_;
  std::vector<bool> scanners_update_;
  std::shared_ptr<PlanarData> latest_scan_data_;
  std::shared_ptr<PFSampleSet> fake_sample_set_;
  std::shared_ptr<ParticleFilter> pf_;
  PFSample fake_sample_;
  PlanarScanner scanner_;
  PlanarModelType model_type_;
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber map_sub_;
  ros::Timer check_scanner_timer_;
  ros::Time latest_scan_received_ts_;
  ros::Duration check_scanner_interval_;
  tf::TransformListener tf_;
  int max_beams_;
  int map_scale_up_factor_;
  int resample_interval_;
  int resample_count_;
  bool first_map_received_;
  bool first_map_only_;
  bool do_beamskip_;
  bool force_update_;  // used to temporarily let amcl update samples even when no motion occurs...
  double beam_skip_distance_, beam_skip_threshold_, beam_skip_error_threshold_;
  double gompertz_a_;
  double gompertz_b_;
  double gompertz_c_;
  double gompertz_input_shift_;
  double gompertz_input_scale_;
  double gompertz_output_shift_;
  double sensor_min_range_;
  double sensor_max_range_;
  double sensor_likelihood_max_dist_;
  double off_map_factor_;
  double z_hit_, z_short_, z_max_, z_rand_, sigma_hit_, lambda_short_;
  double non_free_space_factor_;
  double non_free_space_radius_;
  double global_localization_off_map_factor_;
  double global_localization_non_free_space_factor_;
  bool global_localization_active_;
};

}  // namespace amcl

#endif // AMCL_NODE_NODE_2D_H

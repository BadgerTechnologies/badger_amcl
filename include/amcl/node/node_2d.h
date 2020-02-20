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
/* *************************************************************
 * Desc: AMCL Node for 3D AMCL
 * Author: Tyler Buchman (tyler_buchman@jabil.com)
 ***************************************************************/

#ifndef AMCL_NODE_NODE_2D_H
#define AMCL_NODE_NODE_2D_H

#include <message_filters/subscriber.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/duration.h>
#include <ros/subscriber.h>
#include <ros/time.h>
#include <ros/timer.h>
#include <tf/message_filter.h>

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "amcl/AMCLConfig.h"
#include "map/occupancy_map.h"
#include "pf/pf_vector.h"
#include "sensors/planar_scanner.h"

namespace amcl
{

class Node;

class Node2D
{
public:
  Node2D(Node* node, int map_type, std::mutex& configuration_mutex);
  void reconfigure(amcl::AMCLConfig& config);
  void updateFreeSpaceIndices();
  void globalLocalizationCallback();
  double scorePose(const PFVector& p);
private:
  void scanReceived(const sensor_msgs::LaserScanConstPtr& planar_scan);
  bool updateNodePf(const ros::Time& stamp, int scanner_index, bool* force_publication);
  bool updateScanner(const sensor_msgs::LaserScanConstPtr& planar_scan,
                     int scanner_index, bool* resampled);
  void resampleParticles();
  bool resamplePose(const ros::Time& stamp);
  void getMaxWeightPose(double* max_weight_rtn, PFVector* max_pose);
  bool updatePose(const PFVector& max_pose, const ros::Time& stamp);
  bool isMapInitialized();
  void deactivateGlobalLocalizationParams();
  int getFrameToScannerIndex(const std::string& frame_id);
  void mapMsgReceived(const nav_msgs::OccupancyGridConstPtr& msg);
  void initFromNewMap();
  std::shared_ptr<OccupancyMap> convertMap(const nav_msgs::OccupancyGrid& map_msg);
  void checkScanReceived(const ros::TimerEvent& event);
  bool initFrameToScanner(const std::string& frame_id, tf::Stamped<tf::Pose>* scanner_pose,
                          int* scanner_index);
  void updateScannerPose(const tf::Stamped<tf::Pose>& scanner_pose, int scanner_index);
  bool updateLatestScanData(const sensor_msgs::LaserScanConstPtr& planar_scan, int scanner_index);
  bool getAngleStats(const sensor_msgs::LaserScanConstPtr& planar_scan, double* angle_min,
                     double* angle_increment);
  void samplePlanarScan(const sensor_msgs::LaserScanConstPtr& planar_scan, double angle_min,
                        double angle_increment);
  bool updatePf(const sensor_msgs::LaserScanConstPtr& planar_scan,
                int scanner_index, bool* resampled);
  bool resamplePf(const sensor_msgs::LaserScanConstPtr& planar_scan);


  Node* node_;
  std::shared_ptr<OccupancyMap> map_;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>> scan_sub_;
  std::unique_ptr<tf::MessageFilter<sensor_msgs::LaserScan>> scan_filter_;
  std::string scan_topic_;
  std::string odom_frame_id_;
  std::string base_frame_id_;
  std::string global_frame_id_;
  std::string global_alt_frame_id_;
  std::map<std::string, int> frame_to_scanner_;
  std::mutex& configuration_mutex_;
  std::vector<std::shared_ptr<PlanarScanner>> scanners_;
  std::shared_ptr<std::vector<bool>> scanners_update_;
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
  int map_type_;
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

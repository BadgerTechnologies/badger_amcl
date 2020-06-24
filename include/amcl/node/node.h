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

#ifndef AMCL_NODE_NODE_H
#define AMCL_NODE_NODE_H

#include <algorithm>
#include <cmath>
#include <exception>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <message_filters/subscriber.h>
#include <nav_msgs/Odometry.h>
#include <ros/duration.h>
#include <ros/forwards.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/service_server.h>
#include <ros/single_subscriber_publisher.h>
#include <ros/subscriber.h>
#include <ros/time.h>
#include <ros/timer.h>
#include <std_srvs/Empty.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <yaml-cpp/yaml.h>

#include "badger_amcl/AMCLConfig.h"
#include "map/map.h"
#include "node/node_nd.h"
#include "pf/particle_filter.h"
#include "sensors/odom.h"

namespace badger_amcl
{

// Convenience constants for covariance indices.
constexpr int COVARIANCE_XX = 6 * 0 + 0;
constexpr int COVARIANCE_YY = 6 * 1 + 1;
constexpr int COVARIANCE_AA = 6 * 5 + 5;

// Pose hypothesis
struct PoseHypothesis
{
  // Total weight (weights sum to 1)
  double weight;
  Eigen::Vector3d mean;
  Eigen::Matrix3d covariance;
};

class Node
{
public:
  Node();
  void initFromNewMap(std::shared_ptr<Map> new_map, bool use_init_pose);
  void updateFreeSpaceIndices(std::vector<std::pair<int, int>> fsi);
  void initOdomIntegrator();
  bool getOdomPose(const ros::Time& t, Eigen::Vector3d* map_pose);
  std::string getOdomFrameId();
  std::string getBaseFrameId();
  std::shared_ptr<ParticleFilter> getPfPtr();
  void publishParticleCloud();
  void updatePose(const Eigen::Vector3d& max_hyp_mean, const ros::Time& stamp);
  void updateOdomToMapTransform(const tf2::Transform& odom_to_map);
  bool updatePf(const ros::Time& t, std::vector<bool>& scanners_update, int scanner_index,
                int* resample_count, bool* force_publication, bool* force_update);
  void setPfDecayRateNormal();
  void attemptSavePose();
  void savePoseToServer();
  void savePoseToFile();

private:
  void reconfigureCB(AMCLConfig& config, uint32_t level);
  bool globalLocalizationCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  // Generate a random pose in a free space on the map
  Eigen::Vector3d randomFreeSpacePose();
  Eigen::Vector3d uniformPoseGenerator();
  // Score a single pose with the sensor model using the last sensor data
  double scorePose(const Eigen::Vector3d& p);

  // Initial pose related functions
  void initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
  void initialPoseReceivedInternal(geometry_msgs::PoseWithCovarianceStamped& msg_ptr);
  void handleInitialPose(geometry_msgs::PoseWithCovarianceStamped& msg);
  void resolveFrameId(geometry_msgs::PoseWithCovarianceStamped& msg);
  bool checkInitialPose(const geometry_msgs::PoseWithCovarianceStamped& msg);
  void setMsgCovarianceVals(geometry_msgs::PoseWithCovarianceStamped* msg);
  void transformMsgToTfPose(const geometry_msgs::PoseWithCovarianceStamped& msg, tf2::Transform* pose);
  void setInitialPoseHyp(const geometry_msgs::PoseWithCovarianceStamped& msg, const tf2::Transform& pose);
  void publishInitialPose();
  void newInitialPoseSubscriber(const ros::SingleSubscriberPublisher& single_sub_pub);

  void loadPose();
  void publishPose(const geometry_msgs::PoseWithCovarianceStamped& p);
  void applyInitialPose();
  bool loadPoseFromServer();
  bool loadParamFromServer(std::string param_name, double* val);
  bool loadPoseFromFile();
  YAML::Node loadYamlFromFile();

  // Odometry integrator
  void integrateOdom(const nav_msgs::OdometryConstPtr& msg);
  void calcTfPose(const nav_msgs::OdometryConstPtr& msg, Eigen::Vector3d* pose);
  void calcOdomDelta(const Eigen::Vector3d& pose);
  void resetOdomIntegrator();
  void publishTransform(const ros::TimerEvent& event);

  // Update PF helper functions
  void computeDelta(const Eigen::Vector3d& pose, Eigen::Vector3d* delta);
  void setScannersUpdateFlags(const Eigen::Vector3d& delta, std::vector<bool>& scanners_update, bool* force_update);
  void updateOdom(const Eigen::Vector3d& pose, const Eigen::Vector3d& delta);
  void initOdom(const Eigen::Vector3d& pose, std::vector<bool>& scanners_update, int* resample_count, bool* force_publication);

  std::function<Eigen::Vector3d()> uniform_pose_generator_fn_;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher pose_pub_;
  ros::Publisher absolute_motion_pub_;
  ros::Publisher particlecloud_pub_;
  ros::Publisher alt_pose_pub_;
  ros::Publisher alt_particlecloud_pub_;
  ros::Publisher map_odom_transform_pub_;
  bool publish_initial_pose_at_startup_;
  bool initial_pose_loaded_;
  ros::Publisher initial_pose_pub_;
  ros::Subscriber initial_pose_sub_;
  ros::ServiceServer global_loc_srv_;

  int map_type_;
  std::shared_ptr<Map> map_;
  bool wait_for_occupancy_map_;
  std::shared_ptr<NodeND> node_;

  tf2_ros::TransformBroadcaster tfb_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  bool sent_first_transform_;
  tf2::Transform latest_tf_;
  bool latest_tf_valid_;
  // time for tolerance on the published transform,
  // basically defines how long a map->odom transform is good for
  ros::Duration transform_tolerance_;
  bool tf_broadcast_;
  bool tf_reverse_;

  Odom odom_;
  // parameter for what odom to use
  std::string odom_frame_id_;
  // paramater to store latest odom pose
  tf2::Stamped<tf2::Transform> latest_odom_pose_;
  geometry_msgs::PoseWithCovarianceStamped latest_amcl_pose_;
  ros::Subscriber odom_integrator_sub_;
  bool odom_integrator_enabled_;
  bool odom_integrator_ready_;
  Eigen::Vector3d odom_integrator_last_pose_;
  Eigen::Vector3d odom_integrator_absolute_motion_;
  OdomModelType odom_model_type_;

  // parameter for what base to use
  std::string base_frame_id_;
  std::string global_frame_id_;
  std::string global_alt_frame_id_;

  ros::Duration transform_publish_period_;
  ros::Time save_pose_to_server_last_time_;
  ros::Time save_pose_to_file_last_time_;
  ros::Duration save_pose_to_server_period_;
  ros::Duration save_pose_to_file_period_;
  bool save_pose_;
  std::string saved_pose_filepath_;

  // Particle filter
  std::shared_ptr<ParticleFilter> pf_;
  double pf_err_, pf_z_;
  bool odom_init_;
  Eigen::Vector3d pf_odom_pose_;
  double d_thresh_, a_thresh_;
  PFResampleModelType resample_model_type_;
  int min_particles_, max_particles_;
  std::shared_ptr<PoseHypothesis> initial_pose_hyp_;
  double init_pose_[3];
  double init_cov_[3];
  std::shared_ptr<geometry_msgs::PoseWithCovarianceStamped> last_published_pose_;

  bool first_reconfigure_call_;
  std::mutex configuration_mutex_;
  std::mutex tf_mutex_;
  std::mutex latest_amcl_pose_mutex_;
  dynamic_reconfigure::Server<AMCLConfig> dsrv_;
  AMCLConfig default_config_;
  ros::Timer publish_transform_timer_;

  bool global_localization_active_;
  double global_localization_alpha_slow_, global_localization_alpha_fast_;
  double alpha1_, alpha2_, alpha3_, alpha4_, alpha5_;
  double alpha_slow_, alpha_fast_;
  double uniform_pose_starting_weight_threshold_;
  double uniform_pose_deweight_multiplier_;
  std::vector<std::pair<int, int>> free_space_indices_;
};

}  // namespace amcl

#endif  // AMCL_NODE_NODE_H

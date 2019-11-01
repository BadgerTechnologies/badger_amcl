/*
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
///////////////////////////////////////////////////////////////////////////
//
// Desc: AMCL Node for 3D AMCL
// Author: Tyler Buchman (tyler_buchman@jabil.com)
//
///////////////////////////////////////////////////////////////////////////



#ifndef AMCL_NODE_H
#define AMCL_NODE_H

#include <algorithm>
#include <vector>
#include <map>
#include <cmath>

#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

// Signal handling
#include <signal.h>

#include "map.h"
#include "occupancy_map.h"
#include "octomap.h"
#include "particle_filter.h"
#include "odom.h"
#include "planar_scanner.h"
#include "point_cloud_scanner.h"

#include "ros/assert.h"

// roscpp
#include "ros/ros.h"

// Messages that I need
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/SetMap.h"
#include "std_srvs/Empty.h"
#include <octomap_msgs/Octomap.h>

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

namespace amcl
{

// Pose hypothesis
typedef struct
{
  // Total weight (weights sum to 1)
  double weight;

  // Mean of pose esimate
  PFVector pf_pose_mean;

  // Covariance of pose estimate
  PFMatrix pf_pose_cov;

} AMCLHyp;

class Node
{
  public:
    Node();
    ~Node();

    int process();
    void savePoseToServer();
    void savePoseToFile();

  private:
    // Use a child class to get access to tf2::Buffer class inside of tf_
    struct TransformListenerWrapper : public tf::TransformListener
    {
      inline tf2_ros::Buffer &getBuffer() {return tf2_buffer_;}
    };

    const int INDEX_XX_ = 6*0+0;
    const int INDEX_YY_ = 6*1+1;
    const int INDEX_AA_ = 6*5+5;

    void init2D();
    void init3D();
    void deleteNode2D();
    void deleteNode3D();

    // Score a single pose with the sensor model using the last sensor data
    double scorePose(const PFVector &p);
    double scorePose2D(const PFVector &p);
    double scorePose3D(const PFVector &p);
    // Generate a random pose in a free space on the map
    PFVector randomFreeSpacePose();
    // Pose-generating function used to uniformly distribute particles over
    // the map
    static PFVector uniformPoseGenerator(void* arg);
    static std::vector<std::pair<int,int> > free_space_indices;
    // Callbacks
    bool globalLocalizationCallback(std_srvs::Empty::Request& req,
                                    std_srvs::Empty::Response& res);
    void globalLocalizationCallback2D();
    void globalLocalizationCallback3D();
    void planarScanReceived(const sensor_msgs::LaserScanConstPtr& planar_scan);
    void pointCloudReceived(const sensor_msgs::PointCloud2ConstPtr& point_cloud_scan);
    void initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
    void handleInitialPoseMessage(const geometry_msgs::PoseWithCovarianceStamped& orig_msg);
    void occupancyMapReceived(const nav_msgs::OccupancyGridConstPtr& msg);
    void octoMapReceived(const octomap_msgs::OctomapConstPtr& msg);
    void initFromNewMap();
    void initFromNewOccupancyMap();
    void initFromNewOctoMap();
    void freeMapDependentMemory();
    void freeOccupancyMapDependentMemory();
    void freeOctoMapDependentMemory();
    OccupancyMap* convertMap( const nav_msgs::OccupancyGrid& map_msg );
    OctoMap* convertMap( const octomap_msgs::Octomap& map_msg );
    std::string makeFilepathFromName( const std::string filename );
    void loadPose();
    void publishInitPose();
    bool loadPoseFromServer();
    bool loadPoseFromFile();

    YAML::Node loadYamlFromFile();
    void applyInitialPose();

    double getYaw(tf::Pose& t);

    // Odometry integrator
    void integrateOdom(const nav_msgs::OdometryConstPtr& msg);
    void initOdomIntegrator();
    void resetOdomIntegrator();

    // Helper to get odometric pose from transform system
    bool getOdomPose(tf::Stamped<tf::Pose>& pose,
                     double& x, double& y, double& yaw,
                     const ros::Time& t, const std::string& f);

    void reconfigureCB(amcl::AMCLConfig &config, uint32_t level);
    void reconfigure2D(amcl::AMCLConfig &config);
    void reconfigure3D(amcl::AMCLConfig &config);

    void checkPlanarScanReceived(const ros::TimerEvent& event);
    void checkPointCloudScanReceived(const ros::TimerEvent& event);

    void publishTransform(const ros::TimerEvent& event);

    double normalize(double z);
    double angleDiff(double a, double b);

    std::string scan_topic_;

    tf::TransformBroadcaster* tfb_;

    TransformListenerWrapper* tf_;

    // 2: 2d, 3: 3d, else: none
    int map_type_;

    bool sent_first_transform_;

    tf::Transform latest_tf_;
    bool latest_tf_valid_;

    tf::StampedTransform point_cloud_scanner_to_footprint_tf_;

    //parameter for what odom to use
    std::string odom_frame_id_;

    //paramater to store latest odom pose
    tf::Stamped<tf::Pose> latest_odom_pose_;
    geometry_msgs::PoseWithCovarianceStamped latest_amcl_pose_;

    //parameter for what base to use
    std::string base_frame_id_;
    std::string global_frame_id_;
    std::string global_alt_frame_id_;

    bool first_map_only_;
    int map_scale_up_factor_;

    ros::Duration transform_publish_period_;
    ros::Time save_pose_to_server_last_time_;
    ros::Time save_pose_to_file_last_time_;
    ros::Duration save_pose_to_server_period_;
    ros::Duration save_pose_to_file_period_;

    geometry_msgs::PoseWithCovarianceStamped last_published_pose_;

    Map* map_;

    message_filters::Subscriber<sensor_msgs::LaserScan>* planar_scan_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2>* point_cloud_scan_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan>* planar_scan_filter_;
    tf::MessageFilter<sensor_msgs::PointCloud2>* point_cloud_scan_filter_;
    ros::Subscriber initial_pose_sub_;
    std::vector< PlanarScanner* > planar_scanners_;
    std::vector< PointCloudScanner* > point_cloud_scanners_;
    std::vector< bool > planar_scanners_update_;
    std::vector< bool > point_cloud_scanners_update_;
    std::map< std::string, int > frame_to_planar_scanner_;
    std::map< std::string, int > frame_to_point_cloud_scanner_;

    // Particle filter
    ParticleFilter *pf_;
    double pf_err_, pf_z_;
    bool pf_init_;
    PFVector pf_odom_pose_;
    double d_thresh_, a_thresh_;
    PFResampleModelType resample_model_type_;
    int resample_interval_;
    int resample_count_;
    double sensor_min_range_;
    double sensor_max_range_;

    ros::Subscriber odom_integrator_sub_;
    std::string odom_integrator_topic_;
    bool odom_integrator_ready_;
    PFVector odom_integrator_last_pose_;
    PFVector odom_integrator_absolute_motion_;

    //Nomotion update control
    bool m_force_update;  // used to temporarily let amcl update samples even when no motion occurs...

    Odom* odom_;
    PlanarScanner* planar_scanner_;
    PointCloudScanner* point_cloud_scanner_;

    //time for tolerance on the published transform,
    //basically defines how long a map->odom transform is good for
    ros::Duration transform_tolerance_;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher pose_pub_;
    ros::Publisher absolute_motion_pub_;
    ros::Publisher particlecloud_pub_;
    ros::Publisher alt_pose_pub_;
    ros::Publisher alt_particlecloud_pub_;
    ros::Publisher map_odom_transform_pub_;
    ros::Publisher initial_pose_pub_;
    ros::ServiceServer global_loc_srv_;
    ros::Subscriber initial_pose_sub_old_;
    ros::Subscriber map_sub_;

    AMCLHyp* initial_pose_hyp_;
    bool first_map_received_;
    bool first_reconfigure_call_;

    boost::recursive_mutex configuration_mutex_;
    boost::recursive_mutex tf_mutex_;
    boost::recursive_mutex latest_amcl_pose_mutex_;
    dynamic_reconfigure::Server<amcl::AMCLConfig> *dsrv_;
    amcl::AMCLConfig default_config_;
    ros::Timer check_planar_scanner_timer_;
    ros::Timer check_point_cloud_scanner_timer_;
    ros::Timer publish_transform_timer_;

    int max_beams_, min_particles_, max_particles_;
    double alpha1_, alpha2_, alpha3_, alpha4_, alpha5_;
    double alpha_slow_, alpha_fast_;
    double uniform_pose_starting_weight_threshold_;
    double uniform_pose_deweight_multiplier_;
    bool global_localization_active_;
    double global_localization_alpha_slow_, global_localization_alpha_fast_;
    double z_hit_, z_short_, z_max_, z_rand_, sigma_hit_, lambda_short_;
    double point_cloud_scanner_height_;
    //beam skip related params
    bool do_beamskip_;
    double beam_skip_distance_, beam_skip_threshold_, beam_skip_error_threshold_;
    double sensor_likelihood_max_dist_;
    double planar_gompertz_a_;
    double planar_gompertz_b_;
    double planar_gompertz_c_;
    double planar_gompertz_input_shift_;
    double planar_gompertz_input_scale_;
    double planar_gompertz_output_shift_;
    double off_map_factor_;
    double non_free_space_factor_;
    double non_free_space_radius_;
    double global_localization_off_map_factor_;
    double global_localization_non_free_space_factor_;
    OdomModelType odom_model_type_;
    double init_pose_[3];
    double init_cov_[3];
    PlanarModelType planar_model_type_;
    PointCloudModelType point_cloud_model_type_;
    bool tf_broadcast_;
    bool tf_reverse_;

    double off_object_penalty_factor_;

    bool save_pose_;
    std::string saved_pose_filepath_;

    PlanarData *last_planar_data_;
    PointCloudData *last_point_cloud_data_;
    ros::Time last_planar_scan_received_ts_, last_point_cloud_scan_received_ts_;
    ros::Duration planar_scanner_check_interval_, point_cloud_scanner_check_interval_;
};

}

#endif

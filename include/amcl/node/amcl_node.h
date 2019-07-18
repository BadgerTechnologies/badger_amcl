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
#include "pf.h"
#include "amcl_odom.h"
#include "amcl_laser.h"
#include "amcl_lidar.h"

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

} amcl_hyp_t;

static double
normalize(double z)
{
  return atan2(sin(z),cos(z));
}
static double
angle_diff(double a, double b)
{
  double d1, d2;
  a = normalize(a);
  b = normalize(b);
  d1 = a-b;
  d2 = 2*M_PI - fabs(d1);
  if(d1 > 0)
    d2 *= -1.0;
  if(fabs(d1) < fabs(d2))
    return(d1);
  else
    return(d2);
}

class AmclNode
{
  public:
    AmclNode();
    ~AmclNode();

    int process();
    void savePoseToServer();
    void savePoseToFile();

  private:
    void init2D();
    void init3D();
    void deleteAmclNode2D();
    void deleteAmclNode3D();
    const int INDEX_XX_ = 6*0+0;
    const int INDEX_YY_ = 6*1+1;
    const int INDEX_AA_ = 6*5+5;

    std::string scan_topic_;

    tf::TransformBroadcaster* tfb_;

    // Use a child class to get access to tf2::Buffer class inside of tf_
    struct TransformListenerWrapper : public tf::TransformListener
    {
      inline tf2_ros::Buffer &getBuffer() {return tf2_buffer_;}
    };

    TransformListenerWrapper* tf_;

    // 2: 2d, 3: 3d, else: none
    int map_type_;

    bool sent_first_transform_;

    tf::Transform latest_tf_;
    bool latest_tf_valid_;

    tf::StampedTransform lidar_to_footprint_tf_;

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
    void laserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan);
    void lidarReceived(const sensor_msgs::PointCloud2ConstPtr& lidar_scan);
    void initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
    void handleInitialPoseMessage(const geometry_msgs::PoseWithCovarianceStamped& orig_msg);
    void occupancyMapReceived(const nav_msgs::OccupancyGridConstPtr& msg);
    void octoMapReceived(const octomap_msgs::OctomapConstPtr& msg);
    void initFromNewMap();
    void initFromNewMap2D();
    void initFromNewMap3D();
    void freeMapDependentMemory();
    void freeMapDependentMemory2D();
    void freeMapDependentMemory3D();
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

    ros::Duration gui_publish_period;
    ros::Duration transform_publish_period;
    ros::Time save_pose_to_server_last_time;
    ros::Time save_pose_to_file_last_time;
    ros::Duration save_pose_to_server_period;
    ros::Duration save_pose_to_file_period;

    geometry_msgs::PoseWithCovarianceStamped last_published_pose;

    Map* map_;
    char* mapdata;
    int sx, sy;
    double resolution;

    message_filters::Subscriber<sensor_msgs::LaserScan>* laser_scan_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2>* lidar_scan_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan>* laser_scan_filter_;
    tf::MessageFilter<sensor_msgs::PointCloud2>* lidar_scan_filter_;
    ros::Subscriber initial_pose_sub_;
    std::vector< AMCLLaser* > lasers_;
    std::vector< AMCLLidar* > lidars_;
    std::vector< bool > lasers_update_;
    std::vector< bool > lidars_update_;
    std::map< std::string, int > frame_to_laser_;
    std::map< std::string, int > frame_to_lidar_;

    // Particle filter
    ParticleFilter *pf_;
    double pf_err_, pf_z_;
    bool pf_init_;
    PFVector pf_odom_pose_;
    double d_thresh_, a_thresh_;
    pf_resample_model_t resample_model_type_;
    int resample_interval_;
    int resample_count_;
    double sensor_min_range_;
    double sensor_max_range_;

    // Odometry integrator
    void integrateOdom(const nav_msgs::OdometryConstPtr& msg);
    void initOdomIntegrator();
    void resetOdomIntegrator();
    ros::Subscriber odom_integrator_sub_;
    std::string odom_integrator_topic_;
    bool odom_integrator_ready_;
    PFVector odom_integrator_last_pose_;
    PFVector odom_integrator_absolute_motion_;

    //Nomotion update control
    bool m_force_update;  // used to temporarily let amcl update samples even when no motion occurs...

    AMCLOdom* odom_;
    AMCLLaser* laser_;
    AMCLLidar* lidar_;

    ros::Duration cloud_pub_interval;
    ros::Time last_cloud_pub_time;

    // Helper to get odometric pose from transform system
    bool getOdomPose(tf::Stamped<tf::Pose>& pose,
                     double& x, double& y, double& yaw,
                     const ros::Time& t, const std::string& f);

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

    amcl_hyp_t* initial_pose_hyp_;
    bool first_map_received_;
    bool first_reconfigure_call_;

    boost::recursive_mutex configuration_mutex_;
    boost::recursive_mutex tf_mutex_;
    boost::recursive_mutex latest_amcl_pose_mutex_;
    dynamic_reconfigure::Server<amcl::AMCLConfig> *dsrv_;
    amcl::AMCLConfig default_config_;
    ros::Timer check_laser_timer_;
    ros::Timer check_lidar_timer_;
    ros::Timer publish_transform_timer_;

    int max_beams_, min_particles_, max_particles_;
    double alpha1_, alpha2_, alpha3_, alpha4_, alpha5_;
    double alpha_slow_, alpha_fast_;
    double uniform_pose_starting_weight_threshold_;
    double uniform_pose_deweight_multiplier_;
    bool global_localization_active_;
    double global_localization_alpha_slow_, global_localization_alpha_fast_;
    double z_hit_, z_short_, z_max_, z_rand_, sigma_hit_, lambda_short_;
    double lidar_height_;
    //beam skip related params
    bool do_beamskip_;
    double beam_skip_distance_, beam_skip_threshold_, beam_skip_error_threshold_;
    double sensor_likelihood_max_dist_;
    double laser_gompertz_a_;
    double laser_gompertz_b_;
    double laser_gompertz_c_;
    double laser_gompertz_input_shift_;
    double laser_gompertz_input_scale_;
    double laser_gompertz_output_shift_;
    double off_map_factor_;
    double non_free_space_factor_;
    double non_free_space_radius_;
    double global_localization_off_map_factor_;
    double global_localization_non_free_space_factor_;
    odom_model_t odom_model_type_;
    double init_pose_[3];
    double init_cov_[3];
    laser_model_t laser_model_type_;
    lidar_model_t lidar_model_type_;
    bool tf_broadcast_;
    bool tf_reverse_;

    double off_object_penalty_factor_;

    bool save_pose_;
    std::string saved_pose_filepath_;

    void reconfigureCB(amcl::AMCLConfig &config, uint32_t level);
    void reconfigure2D(amcl::AMCLConfig &config);
    void reconfigure3D(amcl::AMCLConfig &config);

    AMCLLaserData *last_laser_data_;
    AMCLLidarData *last_lidar_data_;
    ros::Time last_laser_received_ts_, last_lidar_received_ts_;
    ros::Duration laser_check_interval_, lidar_check_interval_;
    void checkLaserReceived(const ros::TimerEvent& event);
    void checkLidarReceived(const ros::TimerEvent& event);

    void publishTransform(const ros::TimerEvent& event);
};

}

#endif

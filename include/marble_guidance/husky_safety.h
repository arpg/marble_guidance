#ifndef HUSKY_SAFETY_H
#define HUSKY_SAFETY_H

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <iostream>
#include <random>

#include <boost/version.hpp>
#if ((BOOST_VERSION / 100) % 1000) >= 53
#include <boost/thread/lock_guard.hpp>
#endif
#include <boost/circular_buffer.hpp>

#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/MarkerArray.h>
#include <marble_guidance/TrajList.h>
#include <marble_guidance/MotionCmd.h>
#include <marble_guidance/HuskySafety.h>
#include <marble_guidance/BeaconDetect.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
// #include <lcd_pkg/PoseGraph.h>
#include <tf/tf.h>
#include <math.h>
#include <cv_bridge/cv_bridge.h>

using namespace std;
namespace husky_safety{

class huskySafety {
 public:
    huskySafety(const ros::NodeHandle &node_handle,
                            const ros::NodeHandle &private_node_handle);
    ~huskySafety() = default;

    void init();

    // FUNCTIONS //
    // void odomCb(const nav_msgs::OdometryConstPtr& odom_msg);
    void rplidarScanCb(const sensor_msgs::LaserScan::ConstPtr& scan_msg);
    void processLidarScan();
    void computeSFCommands();
    void generateSafetyBoundary();
    void generateProjectionShapes();
    void determineSafetyState();
    void checkSafetyBoundary(std::vector<float> scan);
    bool haveScan();

    float wrapAngle(float angle);
    float sat(float num, float min_val, float max_val);
    float dist(const geometry_msgs::Point p1, const geometry_msgs::Point p2);



 private:
    // public ros node handle
    ros::NodeHandle nh_;
    // private ros node handle
    ros::NodeHandle pnh_;
    ros::NodeHandle priv_nh;
    std::string node_name_{"node_name"};

    // SUBSCRIBERS //
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_rplidar_;

    ros::Publisher pub_scan_final_;
    ros::Publisher pub_recon_wf_nearness_;
    ros::Publisher pub_sf_nearness_;
    ros::Publisher pub_sf_nearness_cmd_;
    ros::Publisher pub_safety_status_;
    ros::Publisher pub_sf_beacon_detect_;

    bool enable_sf_control_;

    // PARAMETERS
    string vehicle_name_;
    bool debug_;


    bool have_scan_;
    vector<float> scan_ranges_;
    vector<float> scan_ranges_reformat_;
    vector<float> scan_ranges_final_;

    int total_scan_points_;
    int num_scan_points_;
    int scan_start_index_;
    double scan_freq_;
    double scan_limit_;
    ros::Time last_scan_time_;

    vector<float> gamma_vector_;
    vector<vector<float>> cos_proj_array_;
    vector<vector<float>> sin_proj_array_;
    float dg_;

    int total_fourier_terms_;
    int num_sf_fourier_terms_;
    cv::Mat nearness_;
    cv::Mat scan_cvmat_;
    vector<float> nearness_a_;
    vector<float> nearness_b_;
    double sf_thresh_k_;

    int num_sf_clusters_;
    float sf_r_cmd_;
    double sf_k_0_;
    double sf_k_psi_;
    double sf_k_d_;

    double f_dist_;
    double s_dist_;
    vector<float> safety_boundary_;
    int left_corner_index_;
    bool too_close_side_;
    bool too_close_front_;
    double max_sensor_dist_;

    sensor_msgs::LaserScan scan_final_msg_;
    marble_guidance::HuskySafety husky_safety_msg_;

    float min_dist_;
    float min_lidar_dist_;

    bool enable_sf_assist_;
    float sf_r_cmd_temp_;
    float last_sf_r_cmd_;
    double sf_cmd_alpha_;

    // Beacon detect
    marble_guidance::BeaconDetect beacon_detect_msg_;
    int beacon_cluster_min_thresh_;
    int beacon_cluster_max_thresh_;
    int side_;
    int total_close_points_;

	bool started_vehicle_stop_;
    ros::Time vehicle_stop_time_;    

}; // class SimpleNodeClass

}  // namespace demo

#endif  // DEMO_SIMPLE_CLASS_NODE_HPP

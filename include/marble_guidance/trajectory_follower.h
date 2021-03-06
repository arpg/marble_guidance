#ifndef TRAJECTORY_FOLLOWER_H
#define TRAJECTORY_FOLLOWER_H

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
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <lcd_pkg/PoseGraph.h>
#include <marble_guidance/MotionCmd.h>
#include <marble_guidance/TrajList.h>

#include <tf/tf.h>
#include <math.h>

using namespace std;
namespace trajectory_follower{

class trajectoryFollower {
 public:
    trajectoryFollower(const ros::NodeHandle &node_handle,
                            const ros::NodeHandle &private_node_handle);
    ~trajectoryFollower() = default;

    void init();

    // FUNCTIONS //
    void cartoTrajCb(const lcd_pkg::PoseGraphConstPtr& msg);
    void liosamTrajCb(const nav_msgs::PathConstPtr& msg);
    void gtTrajCb(const marble_guidance::TrajListConstPtr& msg);
    void odomCb(const nav_msgs::OdometryConstPtr& odom_msg);
    void findNextLookahead();
    void findNextGTLookahead();
    void publishLookahead();
    void publishMotionCmd();
    void followTrajCb(const std_msgs::BoolConstPtr& follow_traj_msg);
    float dist(const geometry_msgs::Point p1, const geometry_msgs::Point p2);
    bool doLookup();
    bool getGroundTruth();
    void computeCmdVel();
    float wrapAngle(float angle);
    float sat(float num, float min_val, float max_val);

 private:
    // public ros node handle
    ros::NodeHandle nh_;
    // private ros node handle
    ros::NodeHandle pnh_;
    ros::NodeHandle priv_nh;
    std::string node_name_{"node_name"};

    // SUBSCRIBERS //
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_carto_traj_;
    ros::Subscriber sub_liosam_traj_;
    ros::Subscriber sub_gt_traj_;
    ros::Subscriber sub_task_;
    ros::Subscriber sub_follow_traj_;

    // PUBLISHERS //
    ros::Publisher pub_lookahead_;
    ros::Publisher pub_traj_motion_cmd_;
    ros::Publisher pub_cmd_vel_;

    nav_msgs::Odometry odom_;
    geometry_msgs::Point odom_point_;
    geometry_msgs::PointStamped lookahead_point_;

    vector<geometry_msgs::Point> traj_list_points_;

    int loop_rate_;
    bool have_odom_;
    bool have_traj_;

    ros::Time last_joy_msg_time_;

    int last_lookahead_index_;
    bool enable_lookahead_lookup_;
    bool have_current_traj_home_;

    int traj_list_size_;
    double lookahead_dist_short_;
    double lookahead_dist_long_;

    bool enable_ground_truth_;
    int gt_traj_list_size_;
    int last_gt_lookahead_index_;
    bool have_current_gt_traj_home_;
    vector<geometry_msgs::Point> gt_traj_list_points_;

    double current_roll_;
    double current_pitch_;
    double current_heading_;

    double turn_in_place_thresh_;
    double turn_in_place_yawrate_;
    double lookahead_dist_thresh_;
    double yawrate_k0_;
    double yawrate_kd_;
    double yawrate_max_;
    double u_cmd_max_;
    float yawrate_cmd_;
    float u_cmd_;

    bool turn_in_place_;
    marble_guidance::MotionCmd traj_motion_cmd_;
    geometry_msgs::Twist cmd_vel_msg_;




}; // class SimpleNodeClass

}  // namespace demo

#endif  // DEMO_SIMPLE_CLASS_NODE_HPP

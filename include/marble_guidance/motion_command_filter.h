#ifndef motion_command_filter_H
#define motion_command_filter_H

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
#include <std_msgs/Empty.h>
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
#include <marble_guidance/BackupStatus.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
// #include <lcd_pkg/PoseGraph.h>
#include <tf/tf.h>
#include <math.h>

using namespace std;
namespace motion_command_filter{

class motionCommandFilter {
 public:
    motionCommandFilter(const ros::NodeHandle &node_handle,
                            const ros::NodeHandle &private_node_handle);
    ~motionCommandFilter() = default;

    void init();

    // FUNCTIONS //
    void odomCb(const nav_msgs::OdometryConstPtr& odom_msg);
    void pathMotionCmdCb(const marble_guidance::MotionCmdConstPtr& msg);
    void trajMotionCmdCb(const marble_guidance::MotionCmdConstPtr& msg);
    void followTrajCb(const std_msgs::BoolConstPtr& msg);
    void slowDownCb(const std_msgs::BoolConstPtr& msg);
    void backupCmdCb(const marble_guidance::BackupStatusConstPtr& msg);
    void estopCmdCb(const std_msgs::BoolConstPtr& msg);
    void beaconDropCb(const std_msgs::BoolConstPtr& msg);
    void huskySafetyCb(const marble_guidance::HuskySafetyConstPtr& msg);
    void sfNearnessCmdCb(const std_msgs::Float32ConstPtr &msg);
    void filterCommands();
    void lowpassFilterCommands(const geometry_msgs::Twist new_command);
    void publishCommands();
    void checkConnections();
    void determineMotionState();
    void computeBeaconDropMotionCmds();
    geometry_msgs::Twist computeBackupCmd(const geometry_msgs::Point lookahead);
    float wrapAngle(float angle);
    float sat(float num, float min_val, float max_val);
    float dist(const geometry_msgs::Point p1, const geometry_msgs::Point p2);

    enum Constants{
      STARTUP = 0,
      ESTOP = 1,
      PATH_FOLLOW = 2,
      TRAJ_FOLLOW = 3,
      PATH_BACKUP = 4,
      TRAJ_BACKUP = 5,
      PATH_TURN_AROUND = 6,
      TRAJ_TURN_AROUND = 7,
      BEACON_DROP = 8,
      BEACON_MOTION = 9,
      ERROR = 10,
      IDLE = 11,
    };

 private:
    // public ros node handle
    ros::NodeHandle nh_;
    // private ros node handle
    ros::NodeHandle pnh_;
    ros::NodeHandle priv_nh;
    std::string node_name_{"node_name"};

    // SUBSCRIBERS //
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_path_motion_cmd_;
    ros::Subscriber sub_traj_motion_cmd_;
    ros::Subscriber sub_follow_traj_;
    ros::Subscriber sub_backup_cmd_;
    ros::Subscriber sub_estop_cmd_;
    ros::Subscriber sub_husky_safety_;
    ros::Subscriber sub_sf_command_;
    ros::Subscriber sub_beacon_cmd_;

    // PUBLISHERS //
    ros::Publisher pub_cmd_vel_;
    ros::Publisher pub_cmd_vel_stamped_;
    ros::Publisher pub_beacon_deploy_;
    ros::Publisher pub_beacon_deploy_virtual_;

    string vehicle_name_;
    int loop_rate_;
    bool use_stamped_twist_;
    double connection_failure_thresh_;

    // odomCb
    bool have_odom_;
    ros::Time last_odom_time_;
    nav_msgs::Odometry current_odom_;
    geometry_msgs::Point current_pos_;
    double current_roll_;
    double current_pitch_;
    double current_heading_;

    // pathMotionCmdCb
    bool have_path_motion_cmd_;
    ros::Time last_path_time_;
    int path_motion_type_;
    geometry_msgs::Twist path_cmd_vel_;
    geometry_msgs::Point path_lookahead_;

    // trajMotionCmdCb
    bool have_traj_motion_cmd_;
    ros::Time last_traj_time_;
    int traj_motion_type_;
    geometry_msgs::Twist traj_cmd_vel_;
    geometry_msgs::Point traj_lookahead_;

    // followTrajCb
    bool enable_trajectory_following_;

    // backupCmdCb
    bool enable_backup_;

    // estopCmdCb
    bool estop_cmd_;

    // beaconDropCb
    bool beacon_drop_cmd_;
    bool started_beacon_clear_motion_;
    ros::Time beacon_clear_start_time_;
    double beacon_clear_motion_duration_;

    // determineMotionState
    int state_;
    int s_startup_;
    int s_estop_;

    int s_path_follow_;
    int s_traj_follow_;
    int s_path_backup_;
    int s_traj_backup_;
    int s_error_;

    int a_fwd_motion_;
    int a_turnaround_;

    // filterCommands
    geometry_msgs::TwistStamped control_command_msg_stamped_;
    geometry_msgs::Twist control_command_msg_;

    // computeBackupCmd
    double backup_lookahead_dist_;
    double yawrate_k0_;
    double yawrate_kd_;
    double yawrate_max_;
    double u_fwd_cmd_max_;
    double u_back_cmd_max_;
    double yaw_error_k_;

    // Husky Safety
    bool enable_husky_safety_;
    bool too_close_side_;
    bool too_close_front_;
    double close_side_speed_;

    // Small field assist
    bool enable_sf_assist_;
    float sf_r_cmd_;
    bool have_sf_r_cmd_;
    ros::Time last_sf_cmd_time_;

    // lowpassFilterCommands
    bool enable_yaw_rate_filtering_;
    float last_forward_speed_;
    float last_yaw_rate_;
    double u_cmd_lp_filt_const_up_;
    double u_cmd_lp_filt_const_down_;
    double yaw_rate_cmd_lp_filt_const_up_;
    double yaw_rate_cmd_lp_filt_const_down_;

    float min_lidar_dist_;
    double backup_turn_thresh_;

    bool beacon_drop_complete_;
    std_msgs::Bool deploy_beacon_;
    std_msgs::Empty deploy_beacon_virtual_;
    ros::Time beacon_drop_start_time_;
    bool have_initial_settle_time_;
    double beacon_drop_motion_settle_dur_;
    bool start_beacon_drop_turn_;
    float goal_heading_;
    bool backup_close_on_left_;
    bool backup_close_on_right_;
    double close_beacon_turn_angle_;
    bool have_target_heading_;
    bool dropped_beacon_;
    ros::Time beacon_drop_time_;
    bool beacon_estop_;

}; // class SimpleNodeClass

}  // namespace demo

#endif  // DEMO_SIMPLE_CLASS_NODE_HPP

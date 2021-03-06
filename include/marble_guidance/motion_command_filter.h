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
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/MarkerArray.h>
#include <marble_guidance/TrajList.h>
#include <marble_guidance/MotionCmd.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <lcd_pkg/PoseGraph.h>
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
    void backupCmdCb(const std_msgs::BoolConstPtr& msg);
    void filterCommands();
    int getLoopRate();

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

    // PUBLISHERS //
    ros::Publisher pub_cmd_vel_;

    string vehicle_name_;
    int loop_rate_;

    // odomCb
    bool have_odom_;
    nav_msgs::Odometry current_odom_;
    geometry_msgs::Point current_pos_;
    double current_roll_;
    double current_pitch_;
    double current_yaw_;

    // pathMotionCmdCb
    bool have_path_motion_cmd_;
    int path_motion_type_;
    geometry_msgs::Twist path_cmd_vel_;
    geometry_msgs::Point path_lookahead_;

    // trajMotionCmdCb
    bool have_traj_motion_cmd_;
    int traj_motion_type_;
    geometry_msgs::Twist traj_cmd_vel_;
    geometry_msgs::Point traj_lookahead_;

    // followTrajCb
    bool enable_trajectory_following_;

    // backupCmdCb
    bool enable_backup_;



}; // class SimpleNodeClass

}  // namespace demo

#endif  // DEMO_SIMPLE_CLASS_NODE_HPP

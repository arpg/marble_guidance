#ifndef PATH_FOLLOWER_H
#define PATH_FOLLOWER_H

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <iostream>
#include <random>

#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace std;

namespace path_follower{

class pathFollower{
  public:
    pathFollower(const ros::NodeHandle &node_handle,
                            const ros::NodeHandle &private_node_handle);
    ~pathFollower() = default;

    // Functions
    void init();
    void pathCb(const nav_msgs::PathConstPtr& path_msg);
    void odomCb(const nav_msgs::OdometryConstPtr& odom_msg);
    void backupCb(const std_msgs::Bool bool_msg);
    void computeControlCommands();
    void publishCmdMsg();
    void findLookahead(nav_msgs::Path path);
    float distanceTwoPoints3D(geometry_msgs::Point p1, geometry_msgs::Point p2);
    float wrapAngle(float angle);
    float sat(float num, float min_val, float max_val);
    bool ready();

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::NodeHandle priv_nh;
    std::string node_name_{"node_name"};

    // Subsribers
    ros::Subscriber sub_path_;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_backup_;

    // Publishers
    ros::Publisher pub_cmd_;
    ros::Publisher pub_lookahead_point_;

    // Variables
    string vehicle_name_;
    string vehicle_frame_;

    bool debug_;

    nav_msgs::Path current_path_;
    ros::Time last_path_time_;
    bool end_path_;
    bool have_path_;
    bool have_odom_;
    geometry_msgs::Pose lookahead_pose_;
    float lookahead_dist_;
    double lookahead_dist_thresh_;
    geometry_msgs::PointStamped lookahead_point_msg_;

    nav_msgs::Odometry current_odom_;
    geometry_msgs::Point current_pos_;
    double current_roll_;
    double current_pitch_;
    double current_heading_;

    double yawrate_k0_;
    double yawrate_kd_;
    float yawrate_cmd_;
    double yawrate_max_;
    double turn_in_place_thresh_;
    double turn_in_place_yawrate_;
    bool turn_in_place_;
    geometry_msgs::Twist control_commands_msg_;
    float u_cmd_;
    double u_cmd_max_;

    bool enable_backup_;


};

} // namespace

#endif

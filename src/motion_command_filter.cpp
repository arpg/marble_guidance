#include <marble_guidance/motion_command_filter.h>

using namespace std;
namespace motion_command_filter{
motionCommandFilter::motionCommandFilter(const ros::NodeHandle &node_handle,
                                       const ros::NodeHandle &private_node_handle)
      :nh_(node_handle),
       pnh_(private_node_handle){
      this->init();
  }

void motionCommandFilter::init() {

    sub_odom_ = nh_.subscribe("odometry_map", 1, &motionCommandFilter::odomCb, this);
    sub_path_motion_cmd_ = nh_.subscribe("traj_motion_cmd", 1, &motionCommandFilter::pathMotionCmdCb, this);
    sub_traj_motion_cmd_ = nh_.subscribe("path_motion_cmd", 1, &motionCommandFilter::trajMotionCmdCb, this);
    sub_follow_traj_ = nh_.subscribe("follow_traj", 1, &motionCommandFilter::followTrajCb, this);
    sub_backup_cmd_ = nh_.subscribe("enable_backup", 1, &motionCommandFilter::backupCmdCb, this);

    pub_cmd_vel_ = nh_.advertise<geometry_msgs::TwistStamped>("cmd_vel", 10);

    pnh_.param<std::string>("vehicle_name", vehicle_name_,"X1");

    // have_traj_motion_cmd_ = false;
    // have_path_motion_cmd_ = false;
    // have_odom_ = false;

}

void motionCommandFilter::odomCb(const nav_msgs::OdometryConstPtr& odom_msg){

  if(!have_odom_) have_odom_ = true;
  current_odom_ = *odom_msg;
  current_pos_ = current_odom_.pose.pose.position;
  geometry_msgs::Quaternion vehicle_quat_msg = current_odom_.pose.pose.orientation;
  tf::Quaternion vehicle_quat_tf;
  tf::quaternionMsgToTF(vehicle_quat_msg, vehicle_quat_tf);
  tf::Matrix3x3(vehicle_quat_tf).getRPY(current_roll_, current_pitch_, current_yaw_);

}

void motionCommandFilter::pathMotionCmdCb(const marble_guidance::MotionCmdConstPtr& msg){
  if(!have_path_motion_cmd_) have_path_motion_cmd_ = true;
  path_motion_type_ = msg->motion_type;
  path_cmd_vel_ = msg->cmd_vel;
  path_lookahead_ = msg->lookahead_point;
}

void motionCommandFilter::trajMotionCmdCb(const marble_guidance::MotionCmdConstPtr& msg){
  if(!have_traj_motion_cmd_) have_traj_motion_cmd_ = true;
  traj_motion_type_ = msg->motion_type;
  traj_cmd_vel_ = msg->cmd_vel;
  traj_lookahead_ = msg->lookahead_point;
}

void motionCommandFilter::followTrajCb(const std_msgs::BoolConstPtr& msg){
  enable_trajectory_following_ = msg->data;
}

void motionCommandFilter::backupCmdCb(const std_msgs::BoolConstPtr& msg){
  enable_backup_ = msg->data;
}

void motionCommandFilter::filterCommands(){
  // Filter all the incoming commands

}

 // end of class
} // End of namespace nearness

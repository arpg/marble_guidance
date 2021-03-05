#include <marble_guidance/path_follower.h>

namespace path_follower{
pathFollower::pathFollower(const ros::NodeHandle &node_handle,
                                       const ros::NodeHandle &private_node_handle)
      :nh_(node_handle),
       pnh_(private_node_handle){
      this->init();
  }

void pathFollower::init() {

    sub_path_ = nh_.subscribe("path", 1, &pathFollower::pathCb, this);
    sub_odom_ = nh_.subscribe("odometry", 1, &pathFollower::odomCb, this);
    sub_backup_ = nh_.subscribe("enable_backup", 1, &pathFollower::backupCb, this);
    pub_cmd_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    pub_lookahead_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("lookahead_pose", 10);

    pnh_.param("turn_in_place_thresh", turn_in_place_thresh_, 1.0);
    pnh_.param("turn_in_place_yawrate", turn_in_place_yawrate_, 1.0);
    pnh_.param("yawrate_k0", yawrate_k0_, 1.0);
    pnh_.param("yawrate_kd", yawrate_kd_ , 1.0);
    pnh_.param("yawrate_max", yawrate_max_ , 1.0);
    pnh_.param("lookahead_distance_threshold", lookahead_dist_thresh_, 1.0);
    pnh_.param("max_forward_speed", u_cmd_max_, 1.0);
    pnh_.param("enable_debug", debug_, false);

    pnh_.param<string>("vehicle_name", vehicle_name_, "X1");
    vehicle_frame_ = vehicle_name_ + "/base_link";

    control_commands_msg_.header.frame_id = vehicle_frame_;
    lookahead_pose_msg_.header.frame_id = vehicle_name_ + "/map";
    have_path_ = false;
    have_odom_ = false;
    enable_backup_ = false;

}

void pathFollower::findLookahead(nav_msgs::Path path){

  // Find the lookahead point on the current path
  vector<geometry_msgs::PoseStamped> poses = path.poses;
  int l = poses.size();

  float dist;
  vector<float> dist_vec;
  float min_dist = 1000;
  int min_dist_index = 0;
  // Find point on path closest to vehicle
  //ROS_INFO("Current: %f, %f, %f, Path: %f, %f, %f, Dist: %f", current_pos_.x,  current_pos_.y, current_pos_.z,  poses[0].pose.position.x, poses[0].pose.position.y, poses[0].pose.position.z, dist);
  for(int i=0; i<l; i++){
    dist_vec.push_back(distanceTwoPoints3D(current_pos_, poses[i].pose.position));
    if(dist_vec[i] < min_dist){
      min_dist = dist_vec[i];
      min_dist_index = i;
    }
  }

  // Iterate forwards from this point to find lookahead
  bool found_lookahead = false;
  int i = min_dist_index;
  end_path_ = false;
  while(!found_lookahead){
    // If the min distance is greater than the lookahead, we will not find a lookahead
    if(min_dist > lookahead_dist_thresh_){
      lookahead_pose_ = poses[i].pose;
      found_lookahead = true;
    }

    if(dist_vec[i] >= lookahead_dist_thresh_){
      lookahead_pose_ = poses[i].pose;
      found_lookahead = true;
    }
    i++;
    if(i == l){
      // Reached the end of the path
      lookahead_pose_ = poses[i].pose;
      found_lookahead = true;
      end_path_ = true;
    }
  }

  if(debug_){
    lookahead_pose_msg_.header.stamp = ros::Time::now();
    lookahead_pose_msg_.pose = lookahead_pose_;
    pub_lookahead_pose_.publish(lookahead_pose_msg_);
  }

}

void pathFollower::computeControlCommands(){

  // YAWRATE COMMAND
  // Find the lookahead point
  findLookahead(current_path_);

  // Create a yaw rate command from the heading error to the lookahead point
  float relative_lookahead_heading = atan2((lookahead_pose_.position.y - current_pos_.y),(lookahead_pose_.position.x - current_pos_.x));
  float lookahead_angle_error = wrapAngle(relative_lookahead_heading - current_heading_);
  float dist = distanceTwoPoints3D(current_pos_, lookahead_pose_.position);
  //ROS_INFO_THROTTLE(1, "Rel. Heading: %f, Cur. Heading: %f, Angle Err: %f, Dist: %f", relative_lookahead_heading, current_heading_, lookahead_angle_error, dist);

  if(abs(lookahead_angle_error) < turn_in_place_thresh_){
    // Use an exponential attractor to generate yawrate cmds
    yawrate_cmd_ = sat(yawrate_k0_*lookahead_angle_error*exp(-yawrate_kd_*dist), -yawrate_max_, yawrate_max_);
    turn_in_place_ = false;
  } else {
    // Lookahead angle error is larger than threshold, so we should turn in place
    yawrate_cmd_ = turn_in_place_yawrate_;
    turn_in_place_ = true;
  }

  // FORWARD SPEED COMMAND
  if(!turn_in_place_){
    // Slow down if we are approaching the lookahead point
    u_cmd_ = sat(u_cmd_max_*(1 - ((lookahead_dist_thresh_ -  dist)/lookahead_dist_thresh_)), 0.0, u_cmd_max_);
  } else {
    u_cmd_ = 0.0;
  }

  ROS_INFO_THROTTLE(1,"Yawrate cmd: %f, Forward Speed cmd: %f", yawrate_cmd_, u_cmd_);

}

void pathFollower::publishCmdMsg(){

  control_commands_msg_.header.stamp = ros::Time::now();
  control_commands_msg_.twist.linear.x = u_cmd_;
  control_commands_msg_.twist.angular.z = yawrate_cmd_;
  pub_cmd_.publish(control_commands_msg_);

}

void pathFollower::pathCb(const nav_msgs::PathConstPtr& path_msg){

  current_path_ = *path_msg;
  have_path_ = true;
  last_path_time_ = ros::Time::now();

}

void pathFollower::odomCb(const nav_msgs::OdometryConstPtr& odom_msg){

  have_odom_ = true;
  current_odom_ = *odom_msg;
  current_pos_ = current_odom_.pose.pose.position;
  geometry_msgs::Quaternion vehicle_quat_msg = current_odom_.pose.pose.orientation;
  tf::Quaternion vehicle_quat_tf;
  tf::quaternionMsgToTF(vehicle_quat_msg, vehicle_quat_tf);
  tf::Matrix3x3(vehicle_quat_tf).getRPY(current_roll_, current_pitch_, current_heading_);

  //ROS_INFO_THROTTLE(1, "Current Odom - X: %f, Y: %f, Z, %f, Heading: %f", current_pos_.x, current_pos_.y, current_pos_.z, current_heading_);

}

void pathFollower::backupCb(const std_msgs::Bool bool_msg){
  enable_backup_ = bool_msg.data;
}

bool pathFollower::ready(){

  // if((ros::Time::now() - last_path_time_ > some_threshold) && end_path_){
  //   have_path_ = false;
  // }
  return have_path_ && have_odom_;

}

float pathFollower::distanceTwoPoints3D(geometry_msgs::Point p1, geometry_msgs::Point p2){

  return sqrt(pow(p1.x-p2.x, 2) + pow(p1.y-p2.y, 2) + pow(p1.z-p2.z, 2));

}

float pathFollower::wrapAngle(float angle){

    if (angle > M_PI){
        angle -= 2*M_PI;
    } else if( angle < -M_PI){
        angle += 2*M_PI;
    }
    return angle;

}


float pathFollower::sat(float num, float min_val, float max_val){

    if (num >= max_val){
        return max_val;
    } else if( num <= min_val){
         return min_val;
    } else {
      return num;
    }

}

 // end of class
} // End of namespace nearness

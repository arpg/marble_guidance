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
    pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    pub_lookahead_point_ = nh_.advertise<geometry_msgs::PointStamped>("lookahead_point", 10);
    pub_motion_cmd_ = nh_.advertise<marble_guidance::MotionCmd>("motion_cmd", 10);
    pub_curvy_path_ = nh_.advertise<nav_msgs::Path>("curvy_path", 10);

    pnh_.param("turn_in_place_thresh", turn_in_place_thresh_, 1.0);
    pnh_.param("turn_in_place_yawrate", turn_in_place_yawrate_, 1.0);
    pnh_.param("yawrate_k0", yawrate_k0_, 1.0);
    pnh_.param("yawrate_kd", yawrate_kd_ , 1.0);
    pnh_.param("yawrate_max", yawrate_max_ , 1.0);
    pnh_.param("lookahead_distance_threshold", lookahead_dist_thresh_, 1.0);
    pnh_.param("max_forward_speed", u_cmd_max_, 1.0);
    pnh_.param("enable_speed_regulation", enable_speed_regulation_, false);
    pnh_.param("yaw_error_k", yaw_error_k_ , 1.0);
    pnh_.param("enable_debug", debug_, false);
    pnh_.param("stopping_distance", stopping_dist_, 0.25);
    pnh_.param("slow_down_distance", slow_down_dist_, 1.0);
    pnh_.param("sim_start", sim_start_, false);

    pnh_.param<string>("vehicle_name", vehicle_name_, "X1");
    vehicle_frame_ = vehicle_name_ + "/base_link";

    // control_commands_msg_.header.frame_id = vehicle_frame_;
    // lookahead_point_msg_.header.frame_id = vehicle_name_ + "/map";
    lookahead_point_msg_.header.frame_id = "world";
		lookahead_dist_thresh_ = 1.25;
    have_path_ = false;
    have_odom_ = false;
    enable_backup_ = false;
    empty_path_ = false;
    new_path_ = false;
    conditioned_path_.header.frame_id = "world";

    desired_path_point_spacing_ = .1;
    slow_down_dist_ = .75;

    // Curve* curve = new Bezier();

    // curve->set_steps(100); // generate 100 interpolate points between the last 4 way points
    //
    // curve->add_way_point(Vector(1, 1, 0));
    // curve->add_way_point(Vector(2, 3, 0));
    // curve->add_way_point(Vector(3, 2, 0));
    // curve->add_way_point(Vector(4, 6, 0));
    //
    // int node_count = curve->node_count();
    //
    // ROS_INFO("node count: %d", node_count);
    // double x, y, z;
    // for(int i = 0; i < node_count; i++){
    //     x = curve->node(i).x;
    //     y = curve->node(i).y;
    //     z = curve->node(i).z;
    //     ROS_INFO("Point: (%f, %f, %f)", x, y, z);
    // }

}

geometry_msgs::Point pathFollower::interpolatePoints(geometry_msgs::Point point1, geometry_msgs::Point point2){
  // Compute next point along the path with desired linear spacing
  geometry_msgs::Point new_point;
  float dx, dy, dz, dist;

  // Find the 3D unit vector that points from current point1 to point2
  dx = point2.x - point1.x;
  dy = point2.y - point1.y;
  dz = point2.z - point1.z;
  dist = distanceTwoPoints3D(point1, point2);

  // Make sure we hit some minimum distance by introducing a scale
  float scale = 1.0;
  if(dist < 2.0*desired_path_point_spacing_){
    scale = 0.5;
  }

  // Compute a new point along this unit vector at desired spacing
  new_point.x = point1.x + (dx/dist)*desired_path_point_spacing_*scale;
  new_point.y = point1.y + (dy/dist)*desired_path_point_spacing_*scale;
  new_point.z = point1.z + (dz/dist)*desired_path_point_spacing_*scale;

  return new_point;
}

void pathFollower::conditionPath(nav_msgs::Path path){
  vector<geometry_msgs::PoseStamped> path_poses = path.poses;
  int l = path_poses.size();

  conditioned_path_.poses.clear();

  // Do a bezier curve interpolation of the path points
  int num_points = l*20;
  // Curve* curve = new Bezier();
  Curve* curve = new BSpline();
  curve->set_steps(num_points);
  for(int i=0; i < l; i++){
    curve->add_way_point(Vector(path_poses[i].pose.position.x, path_poses[i].pose.position.y, path_poses[i].pose.position.z));
  }

  int num_nodes = curve->node_count();

  // Convert from curve to path type
  nav_msgs::Path curvy_path;
  curvy_path.header.frame_id = "world";
  geometry_msgs::PoseStamped pose;
  for(int i=0; i < num_nodes; i++){
    pose.pose.position.x = curve->node(i).x;
    pose.pose.position.y = curve->node(i).y;
    pose.pose.position.z = curve->node(i).z;
    curvy_path.poses.push_back(pose);
    conditioned_path_.poses.push_back(pose);
  }

  pub_curvy_path_.publish(curvy_path);

}

bool pathFollower::findLookahead(nav_msgs::Path path){

  bool have_lookahead = false;
  if(sim_start_){
    have_path_ = true;
    float attractor_d = sqrt(pow((current_pos_.x), 2) + pow((current_pos_.y), 2));

    lookahead_pose_.position.x = 0.0;
    lookahead_pose_.position.y = 0.0;
    lookahead_pose_.position.z = 0.0;

    if(attractor_d < 0.25){
      sim_start_ = false;
      have_path_ = false;
    }
  } else{
    // Find the lookahead point on the current path
    vector<geometry_msgs::PoseStamped> path_poses = path.poses;
    int l = path_poses.size();

    if(!l){
      ROS_INFO_THROTTLE(1.0, "Empty path...");
      return false;
    }

    float dist;
    for(int i = l-1; i >= 0; i--){
      dist = distanceTwoPoints3D(current_pos_, path_poses[i].pose.position);
      //ROS_INFO("index: %d, dist: %f", i, dist);
      if(dist <= lookahead_dist_thresh_){

        lookahead_pose_ = path_poses[i].pose;
        have_lookahead = true;
				// Publish the lookahead
				lookahead_point_msg_.header.stamp = ros::Time::now();
				lookahead_point_msg_.point = lookahead_pose_.position;
				pub_lookahead_point_.publish(lookahead_point_msg_);
				return have_lookahead;
      }

      if(i == 0){
        ROS_INFO_THROTTLE(1.0, "Error, could not find lookahead on current path.");
        have_lookahead = false;
      }
    }

  }

  return have_lookahead;

}

void pathFollower::computeControlCommands(){

  // Check the path point spacing and fill in large gaps
  if(current_path_.poses.size()){
      ROS_INFO_THROTTLE(1.0,"Conditioning path");
    	conditionPath(current_path_);
  }

  // Find the lookahead point
  if(findLookahead(conditioned_path_)){

    // Create a yaw rate command from the heading error to the lookahead point
    float relative_lookahead_heading = atan2((lookahead_pose_.position.y - current_pos_.y),(lookahead_pose_.position.x - current_pos_.x));
    float lookahead_angle_error = wrapAngle(relative_lookahead_heading - current_heading_);
    float dist = distanceTwoPoints3D(current_pos_, lookahead_pose_.position);

    if(abs(lookahead_angle_error) < turn_in_place_thresh_){
      // Use an exponential attractor to generate yawrate cmds
      yawrate_cmd_ = sat(yawrate_k0_*lookahead_angle_error, -yawrate_max_, yawrate_max_);
      turn_in_place_ = false;
    } else {
      // Lookahead angle error is larger than threshold, so we should turn in place
      yawrate_cmd_ = sat(yawrate_k0_*lookahead_angle_error, -turn_in_place_yawrate_, turn_in_place_yawrate_);
      turn_in_place_ = true;
    }

    // FORWARD SPEED COMMAND
    if(!turn_in_place_){
      // Slow down if we are approaching the lookahead point
      u_cmd_ = sat(u_cmd_max_*(1 - sat( (slow_down_dist_ -  dist)/slow_down_dist_, 0.0, 1.0) ), 0.0, u_cmd_max_);
      if(enable_speed_regulation_){
        u_cmd_ = sat(u_cmd_ - yaw_error_k_*abs(lookahead_angle_error), 0.0, u_cmd_max_);
      }
    } else {
      u_cmd_ = 0.0;
    }

    if((empty_path_) || (dist <= stopping_dist_)){
      u_cmd_ = 0.0;
      yawrate_cmd_ = 0.0;
    }
  } else {
    ROS_INFO_THROTTLE(1.0,"Do not have a current lookahead point.");
    u_cmd_ = 0.0;
    yawrate_cmd_ = 0.0;
  }
}

void pathFollower::publishMotionCmd(){

  cmd_vel_msg_.linear.x = u_cmd_;
  cmd_vel_msg_.angular.z = yawrate_cmd_;
  pub_cmd_vel_.publish(cmd_vel_msg_);

  path_motion_cmd_msg_.cmd_vel = cmd_vel_msg_;
  path_motion_cmd_msg_.motion_type = turn_in_place_;
  path_motion_cmd_msg_.lookahead_point = lookahead_pose_.position;

  pub_motion_cmd_.publish(path_motion_cmd_msg_);

}

void pathFollower::pathCb(const nav_msgs::PathConstPtr& path_msg){

  current_path_ = *path_msg;
  if(!have_path_) have_path_ = true;
  new_path_ = true;
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

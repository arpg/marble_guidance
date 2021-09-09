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

    sub_odom_ = nh_.subscribe("odometry", 1, &motionCommandFilter::odomCb, this);
    sub_path_motion_cmd_ = nh_.subscribe("path_motion_cmd", 1, &motionCommandFilter::pathMotionCmdCb, this);
    sub_traj_motion_cmd_ = nh_.subscribe("traj_motion_cmd", 1, &motionCommandFilter::trajMotionCmdCb, this);
    sub_follow_traj_ = nh_.subscribe("follow_traj", 1, &motionCommandFilter::followTrajCb, this);
    sub_backup_cmd_ = nh_.subscribe("backup_status_msg", 1, &motionCommandFilter::backupCmdCb, this);
    sub_estop_cmd_ = nh_.subscribe("estop_cmd", 1, &motionCommandFilter::estopCmdCb, this);
    sub_husky_safety_ = nh_.subscribe("husky_safety", 1, &motionCommandFilter::huskySafetyCb, this);
    sub_sf_command_ = nh_.subscribe("sf_assist_cmd", 1, &motionCommandFilter::sfAssistCmdCb, this);
    sub_beacon_cmd_ = nh_.subscribe("beacon_drop_cmd", 1, &motionCommandFilter::beaconDropCb, this);
    sub_slow_down_ = nh_.subscribe("slowdown", 1, &motionCommandFilter::slowDownCb, this);

    // pub_cmd_vel_stamped_ = nh_.advertise<geometry_msgs::TwistStamped>("cmd_vel_stamped", 10);
    pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    pub_beacon_deploy_ = nh_.advertise<std_msgs::Bool>("deploy",1);
    pub_beacon_deploy_virtual_ = nh_.advertise<std_msgs::Empty>("deploy_virtual",1);

    pnh_.param<std::string>("vehicle_name", vehicle_name_,"X1");
    pnh_.param("connection_failure_thresh", connection_failure_thresh_, 1.0);
    pnh_.param("enable_husky_safety", enable_husky_safety_, false);
    pnh_.param("enable_small_field_assist", enable_sf_assist_, false);

    pnh_.param("yawrate_k0", yawrate_k0_, 1.0);
    pnh_.param("yawrate_kd", yawrate_kd_ , 1.0);
    pnh_.param("yawrate_max", yawrate_max_ , 1.0);
    pnh_.param("backup_lookahead_distance", backup_lookahead_dist_, 1.0);
    pnh_.param("max_forward_speed", u_fwd_cmd_max_, 0.5);
    pnh_.param("max_backward_speed", u_back_cmd_max_, 0.5);
    pnh_.param("yaw_error_k", yaw_error_k_ , 1.0);

    pnh_.param("fwd_speed_lp_filter_const_up", u_cmd_lp_filt_const_up_, .75);
    pnh_.param("fwd_speed_lp_filter_const_down", u_cmd_lp_filt_const_up_, .95);

    pnh_.param("yaw_rate_lp_filter_const_up", yaw_rate_cmd_lp_filt_const_up_, .85);
    pnh_.param("yaw_rate_lp_filter_const_down", yaw_rate_cmd_lp_filt_const_down_, .95);

    pnh_.param("close_side_speed", close_side_speed_, .1);

    pnh_.param("beacon_clear_motion_duration", beacon_clear_motion_duration_, 1.0);
    pnh_.param("beacon_drop_motion_settle_duration", beacon_drop_motion_settle_dur_, 2.0);
    pnh_.param("close_beacon_turn_angle", close_beacon_turn_angle_, 0.5);

    pnh_.param("slow_down_percent", slow_down_percent_, 25.0);

    slow_down_percent_ /= 100;

    state_ = motionCommandFilter::STARTUP;
    a_fwd_motion_ = 0;
    a_turnaround_ = 1;

    estop_cmd_ = true;
    enable_trajectory_following_ = false;
    enable_backup_ = false;
    //enable_sf_assist_ = false;

    too_close_side_ = false;
    too_close_front_ = false;

    last_forward_speed_ = 0.0;
    last_yaw_rate_ = 0.0;
    enable_yaw_rate_filtering_ = true;

    backup_turn_thresh_ = 1.5707;
    deploy_beacon_.data = true;
    beacon_estop_ = false;

    enable_slow_down_ = false;

}

void motionCommandFilter::odomCb(const nav_msgs::OdometryConstPtr& odom_msg){

  if(!have_odom_) have_odom_ = true;
  last_odom_time_ = ros::Time::now();
  current_odom_ = *odom_msg;
  current_pos_ = current_odom_.pose.pose.position;
  geometry_msgs::Quaternion vehicle_quat_msg = current_odom_.pose.pose.orientation;
  tf::Quaternion vehicle_quat_tf;
  tf::quaternionMsgToTF(vehicle_quat_msg, vehicle_quat_tf);
  tf::Matrix3x3(vehicle_quat_tf).getRPY(current_roll_, current_pitch_, current_heading_);

}

void motionCommandFilter::pathMotionCmdCb(const marble_guidance::MotionCmdConstPtr& msg){
  if(!have_path_motion_cmd_) have_path_motion_cmd_ = true;
  last_path_time_ = ros::Time::now();
  path_motion_type_ = msg->motion_type;
  path_cmd_vel_ = msg->cmd_vel;
  //ROS_INFO("Motion cmd x: %f", path_cmd_vel_.linear.x);
  path_lookahead_ = msg->lookahead_point;
}

void motionCommandFilter::trajMotionCmdCb(const marble_guidance::MotionCmdConstPtr& msg){
  if(!have_traj_motion_cmd_) have_traj_motion_cmd_ = true;
  last_traj_time_ = ros::Time::now();
  traj_motion_type_ = msg->motion_type;
  traj_cmd_vel_ = msg->cmd_vel;
  traj_lookahead_ = msg->lookahead_point;
}

void motionCommandFilter::followTrajCb(const std_msgs::BoolConstPtr& msg){
  enable_trajectory_following_ = msg->data;
}

void motionCommandFilter::slowDownCb(const std_msgs::BoolConstPtr& msg){
  enable_slow_down_ = msg->data;
}

void motionCommandFilter::backupCmdCb(const marble_guidance::BackupStatusConstPtr& msg){
  enable_backup_ = msg->enable_backup;
  backup_close_on_left_ = msg->close_on_left;
  backup_close_on_right_ = msg->close_on_right;
}

void motionCommandFilter::estopCmdCb(const std_msgs::BoolConstPtr& msg){
  ROS_INFO("Received estop command.");
  estop_cmd_ = msg->data;
}

void motionCommandFilter::beaconDropCb(const std_msgs::BoolConstPtr& msg){
  ROS_INFO("Received beacon drop command.");
  beacon_drop_cmd_ = msg->data;
}

void motionCommandFilter::huskySafetyCb(const marble_guidance::HuskySafetyConstPtr& msg){
  too_close_side_ = msg->too_close_side;
  too_close_front_ = msg->too_close_front;
  min_lidar_dist_ = msg->min_dist;
}

void motionCommandFilter::sfAssistCmdCb(const std_msgs::Float32ConstPtr& msg){
  if(!have_sf_r_cmd_) have_sf_r_cmd_ = true;
  last_sf_cmd_time_ = ros::Time::now();
  sf_r_cmd_ = msg->data;
}

void motionCommandFilter::checkConnections(){

    // Check odom connection
    if((ros::Time::now() - last_traj_time_).toSec() > connection_failure_thresh_){
      have_odom_ = false;
      if(state_ != s_startup_){
        ROS_INFO_THROTTLE(2.0,"Error: Lost odometry...");
      }
    }

  // Check path connection
  if((ros::Time::now() - last_path_time_).toSec() > connection_failure_thresh_){
    have_path_motion_cmd_ = false;
    if(state_ != s_startup_){
      ROS_INFO_THROTTLE(2.0,"Error: Lost path commands...");
    }
  }

  // Check traj connection
  if((ros::Time::now() - last_traj_time_).toSec() > connection_failure_thresh_){
    have_traj_motion_cmd_ = false;
    if(state_ != s_startup_){
      ROS_INFO_THROTTLE(2.0,"Error: Lost trajectory commands...");
    }
  }

  // Check for small field commands
  if((ros::Time::now() - last_sf_cmd_time_).toSec() > connection_failure_thresh_){
    have_sf_r_cmd_ = false;
    // if(enable_sf_assist_){
    //   //ROS_INFO_THROTTLE(2.0,"Error: Lost SF assist command...");
    // }
  }

}

void motionCommandFilter::determineMotionState(){

  // Motion state machine based on information received from callbacks

  switch(state_){

    case motionCommandFilter::STARTUP:
      // If we are in startup mode and we have a path motion command and odom
      // we can start following the path
      if(have_odom_ && have_path_motion_cmd_){
        state_ = motionCommandFilter::PATH_FOLLOW;
      }
      break;

    case motionCommandFilter::IDLE:
      if(!enable_trajectory_following_ && have_path_motion_cmd_ && have_odom_){
        state_ = motionCommandFilter::PATH_FOLLOW;
      } else if(enable_trajectory_following_ && have_traj_motion_cmd_ && have_odom_){
        state_ = motionCommandFilter::TRAJ_FOLLOW;
      }
      break;

    case motionCommandFilter::PATH_FOLLOW:

      // If we get a path command to turn around, but enable backup is true
      // switch to backup path following
      if((path_motion_type_ == a_turnaround_) && enable_backup_){
        float relative_lookahead_heading = atan2((path_lookahead_.y - current_pos_.y), (path_lookahead_.x - current_pos_.x));
        float relative_heading_error = abs(wrapAngle(relative_lookahead_heading - current_heading_ ));
        //ROS_INFO("Lookahead: (%f, %f)", path_lookahead_.x, path_lookahead_.y);
        //ROS_INFO("Position: (%f, %f)", current_pos_.x, current_pos_.y);
        //ROS_INFO("")
        //ROS_INFO("Relative heading error: %f", relative_heading_error);
        if(relative_heading_error > backup_turn_thresh_){
          state_ = motionCommandFilter::PATH_BACKUP;
        }
      }

      // If we get a command to start following our trajectory
      // and we have a trajectory following commands
      if(enable_trajectory_following_ && have_traj_motion_cmd_){
        // If we should backup instead of turn in place, switch to
        // trajectory backup state, else turn in place and follower
        // trajectory like normal.
        if(enable_backup_){
          state_ = motionCommandFilter::TRAJ_BACKUP;
        } else {
          state_ = motionCommandFilter::TRAJ_FOLLOW;
        }
      }
      break;

    case motionCommandFilter::TRAJ_FOLLOW:
      // If we are trajectory following and enable_trajectory_following
      // changes to false, switch back to path following.
      if(!enable_trajectory_following_ && have_path_motion_cmd_){
        state_ = motionCommandFilter::PATH_FOLLOW;
      }
      // If we get a traj command to turn around, but enable backup is true
      // switch to traj backup mode
      if((traj_motion_type_ == a_turnaround_) && enable_backup_){
        state_ = motionCommandFilter::TRAJ_BACKUP;
      }
      break;

    case motionCommandFilter::PATH_BACKUP:
    {
      if(!enable_backup_){
        state_ = motionCommandFilter::PATH_TURN_AROUND;
      }
      float relative_lookahead_heading = atan2((path_lookahead_.y - current_pos_.y), (path_lookahead_.x - current_pos_.x));
      float relative_heading_error = abs(wrapAngle(relative_lookahead_heading - current_heading_ ));
      if(abs(relative_heading_error) < M_PI/2.0){
        // The path has switched and is now in front of us
        // No need to backup any further or turn around in palce
        state_ = motionCommandFilter::PATH_FOLLOW;
      }
    }
      break;

    case motionCommandFilter::TRAJ_BACKUP:
      if(!enable_backup_){
        state_ = motionCommandFilter::TRAJ_TURN_AROUND;
      }
      break;

    case motionCommandFilter::PATH_TURN_AROUND:
      if(path_motion_type_ == a_fwd_motion_){
        state_ = motionCommandFilter::PATH_FOLLOW;
      }
      break;

    case motionCommandFilter::TRAJ_TURN_AROUND:
      if(traj_motion_type_ == a_fwd_motion_){
        state_ = motionCommandFilter::TRAJ_FOLLOW;
      }
      break;

    case motionCommandFilter::ESTOP:
      if(!estop_cmd_){
        state_ = motionCommandFilter::IDLE;
      }
      break;

    case motionCommandFilter::BEACON_DROP:
      // If we are in the beacon drop state, check if we can place the beacon
      // off of the current trajectory

      if(beacon_drop_complete_){
        state_ = motionCommandFilter::BEACON_MOTION;
        started_beacon_clear_motion_ = false;
      }

      // if(!beacon_drop_cmd_){
      //   state_ = motionCommandFilter::BEACON_MOTION;
      //   started_beacon_clear_motion_ = false;
      // }
      break;

    case motionCommandFilter::BEACON_MOTION:
      // If we are in the beacon motion state, move forward until the beacon is cleared.

      // Do a check to see how long we have been in this state
      if(!started_beacon_clear_motion_){
        beacon_clear_start_time_ = ros::Time::now();
        started_beacon_clear_motion_ = true;
      }

      if((ros::Time::now() - beacon_clear_start_time_).toSec() > beacon_clear_motion_duration_){
        // End of beacon drop dance, go back to Idle.
        state_ = motionCommandFilter::IDLE;
      }
      break;

  }

  // No matter what state we are in, switch to s_estop
  // if we receive an estop command.
  if(estop_cmd_ && !(state_ == motionCommandFilter::ESTOP || state_ == motionCommandFilter::BEACON_DROP || state_ == motionCommandFilter::BEACON_MOTION)){
    ROS_INFO("Setting estop state");
    state_ = motionCommandFilter::ESTOP;
    control_command_msg_.linear.x = 0.0;
    control_command_msg_.angular.z = 0.0;
    pub_cmd_vel_.publish(control_command_msg_);
  }

  if(beacon_drop_cmd_ && !(state_ == motionCommandFilter::BEACON_DROP || state_ == motionCommandFilter::BEACON_MOTION)){
    state_ = motionCommandFilter::BEACON_DROP;
    beacon_drop_start_time_ = ros::Time::now();
    beacon_drop_complete_ = false;
    beacon_drop_cmd_ = false;
    start_beacon_drop_turn_ = false;
    have_initial_settle_time_ = false;
    have_target_heading_ = false;
    dropped_beacon_ = false;
    control_command_msg_.linear.x = 0.0;
    control_command_msg_.angular.z = 0.0;
    pub_cmd_vel_.publish(control_command_msg_);
  }

}

void motionCommandFilter::filterCommands(){
  // Filter all the incoming commands

  switch(state_){

    case motionCommandFilter::STARTUP:
      ROS_INFO_THROTTLE(0.5,"Motion filter: startup");
      control_command_msg_.linear.x = 0.0;
      control_command_msg_.angular.z = 0.0;
      break;

    case motionCommandFilter::IDLE:
      ROS_INFO_THROTTLE(2.0,"Motion filter: idle");
      control_command_msg_.linear.x = 0.0;
      control_command_msg_.angular.z = 0.0;
      break;

    case motionCommandFilter::ESTOP:
      ROS_INFO_THROTTLE(2.0,"Motion filter: estop");
      control_command_msg_.linear.x = 0.0;
      control_command_msg_.angular.z = 0.0;
      break;

    case motionCommandFilter::PATH_FOLLOW:
      ROS_INFO_THROTTLE(2.0,"Motion filter: path follow");
      control_command_msg_ = path_cmd_vel_;
      if(enable_husky_safety_){

        if(min_lidar_dist_ < 2.0 && path_cmd_vel_.linear.x != 0.0){
          ROS_INFO_THROTTLE(2.0,"Getting close to obstacle");
          control_command_msg_.linear.x = sat(path_cmd_vel_.linear.x*pow(min_lidar_dist_,2), 0.1, u_fwd_cmd_max_);
        }
        // Regulate vehicle forward speed based on safety limits
        if(too_close_side_&& path_cmd_vel_.linear.x != 0.0){
          ROS_INFO_THROTTLE(2.0,"Too close on the side!");
          control_command_msg_.linear.x = close_side_speed_;
        }
        // Need to stop if we detect something in the front of the vehicle
        if(too_close_front_){
          ROS_INFO_THROTTLE(2.0,"Too close in front!");
          control_command_msg_.linear.x = 0.0;
        }

      }
      if(enable_sf_assist_){
        if(have_sf_r_cmd_){
          control_command_msg_.angular.z += sf_r_cmd_;
        }
      }
      break;

    case motionCommandFilter::TRAJ_FOLLOW:
      ROS_INFO_THROTTLE(2.0,"Motion filter: trajectory follow");
      control_command_msg_ = traj_cmd_vel_;
      if(enable_husky_safety_){
        // Regulate vehicle forward speed based on safety limits
        if(too_close_side_){
          control_command_msg_.linear.x = close_side_speed_;
        }
        // Need to stop if we detect something in the front of the vehicle
        if(too_close_front_){
          control_command_msg_.linear.x = 0.0;
        }
      }
      if(enable_sf_assist_){
        if(have_sf_r_cmd_){
          control_command_msg_.angular.z += sf_r_cmd_;
        }
      }
      break;

    case motionCommandFilter::PATH_BACKUP:
      ROS_INFO_THROTTLE(2.0,"Motion filter: path backup");
      control_command_msg_ = computeBackupCmd(path_lookahead_);
      break;

    case motionCommandFilter::TRAJ_BACKUP:
      ROS_INFO_THROTTLE(2.0,"Motion filter: trajectory backup");
      control_command_msg_ = computeBackupCmd(traj_lookahead_);
      break;

    case motionCommandFilter::PATH_TURN_AROUND:
      ROS_INFO_THROTTLE(2.0,"Motion filter: path turning around");
      control_command_msg_ = path_cmd_vel_;
      break;

    case motionCommandFilter::TRAJ_TURN_AROUND:
      ROS_INFO_THROTTLE(2.0,"Motion filter: traj turning around");
      control_command_msg_ = traj_cmd_vel_;
      break;

    case motionCommandFilter::BEACON_DROP:
      ROS_INFO_THROTTLE(2.0, "Motion filter: beacon drop");
      computeBeaconDropMotionCmds();
      break;

    case motionCommandFilter::BEACON_MOTION:
      ROS_INFO_THROTTLE(2.0, "Motion filter: clearing dropped beacon");
      control_command_msg_.linear.x = 0.2;
      control_command_msg_.angular.z = 0.0;
      // Need to stop if we detect something in the front of the vehicle
      if(too_close_front_){
        ROS_INFO_THROTTLE(2.0,"Too close in front!");
        control_command_msg_.linear.x = 0.0;
      }
      break;

    case motionCommandFilter::ERROR:
      ROS_INFO_THROTTLE(2.0,"Motion filter: error");
      control_command_msg_.linear.x = 0.0;
      control_command_msg_.angular.z = 0.0;
      break;
  }

}

geometry_msgs::Twist motionCommandFilter::computeBackupCmd(const geometry_msgs::Point lookahead){

  float u_cmd;
  float yawrate_cmd;

  // Create a yaw rate command from the heading error to the lookahead point
  float relative_lookahead_heading = atan2((lookahead.y - current_pos_.y), (lookahead.x - current_pos_.x));
  float lookahead_angle_error = wrapAngle(relative_lookahead_heading - (current_heading_ + M_PI));
  float distance = dist(current_pos_, lookahead);
  //ROS_INFO_THROTTLE(1, "Rel. Heading: %f, Cur. Heading: %f, Angle Err: %f, Dist: %f", relative_lookahead_heading, current_heading_, lookahead_angle_error, dist);

  // Use an exponential attractor to generate yawrate cmds
  yawrate_cmd = sat(yawrate_k0_*lookahead_angle_error*exp(-yawrate_kd_*distance), -yawrate_max_, yawrate_max_);

  // FORWARD SPEED COMMAND
  // Slow down if we are approaching the lookahead point

  u_cmd = -sat(u_back_cmd_max_*(1 - ((backup_lookahead_dist_ -  distance)/backup_lookahead_dist_)), 0.05, u_back_cmd_max_);
  u_cmd = sat(u_cmd + (yaw_error_k_/2.0)*abs(lookahead_angle_error), -u_back_cmd_max_, -.05);

  if (distance < 0.5){
    u_cmd = 0.0;
    yawrate_cmd = 0.0;
  }

  geometry_msgs::Twist backup_cmd;
  backup_cmd.linear.x = u_cmd;
  backup_cmd.angular.z = yawrate_cmd;

  return backup_cmd;

}

void motionCommandFilter::computeBeaconDropMotionCmds(){
  // First, check to see how much space we have around us.
  // For now, just use the backup detector

  // A better option would be to figure out which side we are
  // close to a wall on, and do a 30 degree turn prior to dropping.
  control_command_msg_.linear.x = 0.0;
  control_command_msg_.angular.z = 0.0;

  if(estop_cmd_){
    // We received an estop command, don't do anything until it's cleared
    beacon_estop_ = true;
  } else {
    // Need to reset timers accordingly
    if(beacon_estop_){
      if(have_initial_settle_time_){
        beacon_drop_start_time_ = ros::Time::now();
        if(dropped_beacon_){
          beacon_drop_time_ = ros::Time::now();
        }
      }
    beacon_estop_ = false;
    }
  }

  if(!beacon_estop_){
    if(!have_target_heading_){

      if(enable_backup_){
        // We are already close to a wall, just drop the beacon
        // Give the vehicle a few seconds to settle to a stop.
        if(backup_close_on_left_){
          // We are close to an obstacle on the left, only turn 30 degrees cw
          goal_heading_ = wrapAngle(current_heading_ - close_beacon_turn_angle_);
        } else if(backup_close_on_right_){
          // We are close to an obstacle on the right, only turn 30 degrees ccw
          goal_heading_ = wrapAngle(current_heading_ + close_beacon_turn_angle_);
        }

      } else {
        // We have enough room to do a 90 degree turn
        goal_heading_ = wrapAngle(current_heading_ + M_PI/2.0);
      }

      have_target_heading_ = true;
    }

    float heading_error = wrapAngle(goal_heading_ - current_heading_);
    if(abs(heading_error) >= .1){
      control_command_msg_.linear.x = 0.0;
      control_command_msg_.angular.z = sat(yawrate_k0_*heading_error, -yawrate_max_, yawrate_max_);
    } else {
      // We have finished turning, wait for the vehicle to settle
      if(!have_initial_settle_time_){
        beacon_drop_start_time_ = ros::Time::now();
        have_initial_settle_time_ = true;
      }
      float dur = (ros::Time::now() - beacon_drop_start_time_).toSec();
      if(dur >= beacon_drop_motion_settle_dur_){
        if(!dropped_beacon_){
          beacon_drop_time_ = ros::Time::now();
          pub_beacon_deploy_.publish(deploy_beacon_);
          pub_beacon_deploy_virtual_.publish(deploy_beacon_virtual_);
          dropped_beacon_ = true;
        } else {
          float drop_dur =  (ros::Time::now() - beacon_drop_time_).toSec();
          if(drop_dur > 2.0){
            beacon_drop_complete_ = true;
          }
        }
      }
    }
  }

}

void motionCommandFilter::publishCommands(){

  // if(use_stamped_twist_){
  //   control_command_msg_stamped_.header.stamp = ros::Time::now();
  //   control_command_msg_stamped_.twist = control_command_msg_;
  //   pub_cmd_vel_stamped_.publish(control_command_msg_stamped_);
  // } else {
  if(!estop_cmd_){

    if(enable_slow_down_){
      control_command_msg_.linear.x *= slow_down_percent_;
      control_command_msg_.angular.z *= slow_down_percent_;
    }

    lowpassFilterCommands(control_command_msg_);

    pub_cmd_vel_.publish(control_command_msg_);
  }

}

void motionCommandFilter::lowpassFilterCommands(const geometry_msgs::Twist new_command){
  // We want different filters for speeding up vs slowing down
    bool backup = ( state_ == motionCommandFilter::PATH_BACKUP || state_ == motionCommandFilter::TRAJ_BACKUP);
    float back_const_up_ = .75;
    float back_const_down_ = .85;
    if(new_command.linear.x > last_forward_speed_){
      if(!backup){
        control_command_msg_.linear.x = u_cmd_lp_filt_const_up_*last_forward_speed_ + (1.0 - u_cmd_lp_filt_const_up_)*new_command.linear.x;
      } else {
        control_command_msg_.linear.x = back_const_down_*last_forward_speed_ + (1.0 - back_const_down_)*new_command.linear.x;
      }
      last_forward_speed_ = control_command_msg_.linear.x;
    } else if (new_command.linear.x < last_forward_speed_){
      if(!backup){
        control_command_msg_.linear.x = u_cmd_lp_filt_const_down_*last_forward_speed_ + (1.0 - u_cmd_lp_filt_const_down_)*new_command.linear.x;
      } else {
        control_command_msg_.linear.x = back_const_up_*last_forward_speed_ + (1.0 - back_const_up_)*new_command.linear.x;
      }
      last_forward_speed_ = control_command_msg_.linear.x;
    }

  if(enable_yaw_rate_filtering_){
    if(new_command.angular.z > last_yaw_rate_){
      control_command_msg_.angular.z = yaw_rate_cmd_lp_filt_const_up_*last_yaw_rate_ + (1.0 - yaw_rate_cmd_lp_filt_const_up_)*new_command.angular.z;
      last_yaw_rate_ = control_command_msg_.angular.z;
    } else if (new_command.angular.z < last_yaw_rate_){
      control_command_msg_.angular.z = yaw_rate_cmd_lp_filt_const_down_*last_yaw_rate_ + (1.0 - yaw_rate_cmd_lp_filt_const_down_)*new_command.angular.z;
      last_yaw_rate_ = control_command_msg_.angular.z;
    }
  }

  // Need to hard stop if commanded
  if(new_command.linear.x == 0.0){
    control_command_msg_.linear.x = 0.0;
    last_forward_speed_ = 0.0;
  }
  // Need to hard stop if commanded
  if(new_command.angular.z == 0.0){
    control_command_msg_.angular.z = 0.0;
    last_yaw_rate_ = 0.0;
  }
}


float motionCommandFilter::dist(const geometry_msgs::Point p1, const geometry_msgs::Point p2)
{
    float distance = sqrt(pow((p1.x - p2.x),2) + pow((p1.y - p2.y),2));
    return distance;
}

float motionCommandFilter::wrapAngle(float angle){

    if (angle > M_PI){
        angle -= 2*M_PI;
    } else if( angle < -M_PI){
        angle += 2*M_PI;
    }
    return angle;
}

float motionCommandFilter::sat(float num, float min_val, float max_val){

    if (num >= max_val){
        return max_val;
    } else if( num <= min_val){
         return min_val;
    } else {
      return num;
    }
}

 // end of class
} // End of namespace motion_command_filter

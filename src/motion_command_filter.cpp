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
    sub_sf_command_ = nh_.subscribe("sf_nearness_cmd", 1, &motionCommandFilter::sfNearnessCmdCb, this);
    sub_beacon_cmd_ = nh_.subscribe("beacon_drop_cmd", 1, &motionCommandFilter::beaconDropCb, this);
    sub_stair_mode_ = nh_.subscribe("stair_mode_cmd", 1, &motionCommandFilter::stairModeCb, this);

    // pub_cmd_vel_stamped_ = nh_.advertise<geometry_msgs::TwistStamped>("cmd_vel_stamped", 10);
    pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    pub_beacon_deploy_ = nh_.advertise<std_msgs::Bool>("deploy",1);
    pub_beacon_deploy_virtual_ = nh_.advertise<std_msgs::Empty>("deploy_virtual",1);
    pub_planning_task_ = nh_.advertise<std_msgs::String>("planning_task",1);

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

    pnh_.param("is_spot", is_spot_, false);
    pnh_.param("offline_test", offline_test_, false);
    pnh_.param("stair_mode_pitch_thresh", stair_mode_pitch_thresh_, 0.25);
    pnh_.param("stair_align_thresh", stair_align_thresh_, 0.15);
    pnh_.param("stair_turnaround_thresh", stair_turnaround_thresh_, 0.15);
    pnh_.param("stair_mode_forward_speed", stair_mode_fwd_speed_, 0.5);

    pnh_.param("z_diff_thresh", z_diff_thresh_, 0.25);
    pnh_.param("stair_pause_time", stair_pause_time_s_, 5.0);

    pnh_.param<string>("vehicle_name", vehicle_name_, "X1");

    string stair_client_string = "/" + vehicle_name_ + "/spot/stair_mode";

    stair_mode_client_ = nh_.serviceClient<std_srvs::SetBool>(stair_client_string);

    state_ = motionCommandFilter::STARTUP;
    a_fwd_motion_ = 0;
    a_turnaround_ = 1;

    stair_mode_cmd_ = false;
    estop_cmd_ = false;
    enable_trajectory_following_ = false;
    enable_backup_ = false;
    enable_sf_assist_ = false;

    too_close_side_ = false;
    too_close_front_ = false;

    last_forward_speed_ = 0.0;
    last_yaw_rate_ = 0.0;
    enable_yaw_rate_filtering_ = true;

    backup_turn_thresh_ = 1.5707;
    deploy_beacon_.data = true;
    beacon_estop_ = false;

    planning_link_z_offset_ = .5;

    beacon_drop_cmd_ = false;
    have_new_path_ = false;
    path_goal_point_.x = 0.0;
    path_goal_point_.y = 0.0;
    path_goal_point_.z = 0.0;
    last_path_goal_point_ = path_goal_point_;

    is_stair_mode_on_ = false;
    is_up_stairs_ = false;
    is_down_stairs_ = false;

    stair_goal_point_offset_ = 0.5;
    planning_task_.data = "eop";

    end_stair_pause_ = false;
    stair_pause_complete_ = false;

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
  stair_lookahead_ = msg->lookahead_point_stairs;
  path_goal_point_ = msg->goal_point;
  // ROS_INFO("lookahead_x: %f, gp_x: %f", path_goal_point_.x, last_path_goal_point_.x);
  // float gp_dist = 0.0;
  float gp_dist = dist(path_goal_point_, last_path_goal_point_);
  // ROS_INFO("gp dist: %f", gp_dist);
  if( gp_dist > .01){
    have_new_path_ = true;
    last_path_goal_point_ = path_goal_point_;
  }
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

void motionCommandFilter::stairModeCb(const std_msgs::BoolConstPtr& msg){
  // ROS_INFO("Received stair mode command.");
  stair_mode_cmd_ = msg->data;
}

void motionCommandFilter::huskySafetyCb(const marble_guidance::HuskySafetyConstPtr& msg){
  too_close_side_ = msg->too_close_side;
  too_close_front_ = msg->too_close_front;
  min_lidar_dist_ = msg->min_dist;
}

void motionCommandFilter::sfNearnessCmdCb(const std_msgs::Float32ConstPtr& msg){
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
    if(enable_sf_assist_){
      ROS_INFO_THROTTLE(2.0,"Error: Lost SF assist command...");
    }
  }

}

void motionCommandFilter::determineMotionState(){

  // Handle stair mode seperately from everything else
  //if(!stair_mode_cmd_ && is_stair_mode_on_ && !(state_ == motionCommandFilter::STAIR_MODE_UP || state_ == motionCommandFilter::STAIR_MODE_DOWN)){
  if(!stair_mode_cmd_ && is_stair_mode_on_ && !checkStairProgress()){
    stair_mode_srv_.request.data = false;
    if(stair_mode_client_.call(stair_mode_srv_) || offline_test_){
    //if(true){
      is_stair_mode_on_ = false;
      ROS_INFO("Motion filter: Spot stair mode disengaged.");
      ROS_INFO("Motion filter: Not on stairs, switching back to IDLE.");
      state_ = motionCommandFilter::IDLE;
    } else {
      ROS_INFO_THROTTLE(1.0, "Motion filter: Spot stair mode disengage failed...");
    }
  }


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
      if(!enable_trajectory_following_ && have_path_motion_cmd_ && have_new_path_ && have_odom_){
        state_ = motionCommandFilter::PATH_FOLLOW;
      } else if(enable_trajectory_following_ && have_traj_motion_cmd_ && have_odom_){
        state_ = motionCommandFilter::TRAJ_FOLLOW;
      }
      break;

    case motionCommandFilter::PATH_FOLLOW:

      // If we get a path command to turn around, but enable backup is true
      // switch to backup path following
      if((path_motion_type_ == a_turnaround_) && enable_backup_ && !is_spot_){
        float relative_lookahead_heading = atan2((path_lookahead_.y - current_pos_.y), (path_lookahead_.x - current_pos_.x));
        float relative_heading_error = abs(wrapAngle(relative_lookahead_heading - current_heading_ ));

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

      if(is_spot_ && stair_mode_cmd_){
        //ROS_INFO("ENTERING STAIR MODE...");
        // if the path is upstairs, switch to stair mode and start following
        //if(isUpstairs(path_goal_poin_)){
        if(isUpstairs(stair_lookahead_)){
          ROS_INFO("ENTERING STAIR MODE UP");
          is_up_stairs_ = false;
          align_heading_error_ = 0.0;
          stair_mode_srv_.request.data = true;
          end_stair_pause_ = false;
          stair_pause_complete_ = false;
          if(stair_mode_client_.call(stair_mode_srv_) || offline_test_){
            if(!is_stair_mode_on_) is_stair_mode_on_ = true;
            state_ = motionCommandFilter::STAIR_MODE_UP;
            started_stairs_ = false;
	          start_stair_z_ = current_pos_.z;
            ROS_INFO("Motion filter: Spot stair mode engaged.");
            float relative_lookahead_heading = atan2((path_lookahead_.y - current_pos_.y), (path_lookahead_.x - current_pos_.x));
            align_heading_error_ = abs(wrapAngle(relative_lookahead_heading - current_heading_ ));
            if(align_heading_error_ > stair_align_thresh_){
              ROS_INFO("DO STAIR ALIGN");
              do_stair_align_ = true;
            } else {
              ROS_INFO("NOT ALIGNING");
            }
          } else {
            ROS_INFO_THROTTLE(1.0, "Motion filter: Spot stair mode engage failed...");
            // NEED TO HANDLE THIS SOMEHOW
            // Might be handled by blacklisting
          }

        //} else if (isDownstairs(path_goal_point_)) {
        } else if (isDownstairs(stair_lookahead_)) {
          ROS_INFO("ENTERING STAIR MODE DOWN");
          started_stairs_ = false;
          is_down_stairs_ = false;
          end_stair_pause_ = false;
          stair_pause_complete_ = false;
          stair_mode_srv_.request.data = true;
          if(stair_mode_client_.call(stair_mode_srv_) || offline_test_){
          //if(true){
            if(!is_stair_mode_on_) is_stair_mode_on_ = true;
	          start_stair_z_ = current_pos_.z;
            // Path is down stairs, need to turn around first
            state_ = motionCommandFilter::STAIR_MODE_DOWN;
            float relative_lookahead_heading = atan2((path_lookahead_.y - current_pos_.y), (path_lookahead_.x - current_pos_.x));
            align_heading_error_ = abs(wrapAngle(relative_lookahead_heading - (current_heading_ + M_PI )));
            if(align_heading_error_ > stair_turnaround_thresh_){
              do_stair_turnaround_ = true;
              ROS_INFO("DO STAIR ALIGN");
            } else {
              ROS_INFO("NOT ALIGNING");
            }
          } else {
            ROS_INFO_THROTTLE(1.0, "Motion filter: Spot stair mode engage failed...");
            // NEED TO HANDLE THIS SOMEHOW
            // Might be handled by blacklisting
          }
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

    case motionCommandFilter::STAIR_MODE_UP:
      // Need to go up the stairs, but do not need to turn around first
      // ROS_INFO_THROTTLE(1.0,"STAIRS MODE...");

      if(do_stair_align_){
        ROS_INFO_THROTTLE(1.0,"ALIGNING WITH THE STAIRS...");
        float relative_lookahead_heading = atan2((path_lookahead_.y - current_pos_.y), (path_lookahead_.x - current_pos_.x));
        align_heading_error_ = abs(wrapAngle(relative_lookahead_heading - current_heading_));
        //ROS_INFO_THROTTLE(.5,"error: %f", align_heading_error_);
        if(align_heading_error_ < stair_align_thresh_){
          do_stair_align_ = false;
          ROS_INFO("ALIGNING WITH STAIRS COMPLETE");
        }
      } else {
        // ROS_INFO_THROTTLE(1.0,"CHECKING PROGRESS...");
        if(checkStairProgress()){
          // ROS_INFO("TOP OF STAIRS REACHED, EXITING STAIR MODE UP");
          ROS_INFO_THROTTLE(1.0, "TOP OF STAIRS REACHED");
          // Need to stay in stair mode until we leave area
          float path_gp_dist = dist(path_goal_point_, current_pos_);

          // We need to pause for a little bit so we don't go right back down the stairs
          if(!end_stair_pause_){
            end_stair_pause_ = true;
            end_stair_time_ = ros::Time::now();
          }

          float stair_pause_time_diff_s = (ros::Time::now() - end_stair_time_).toSec();
          if(stair_pause_time_diff_s > stair_pause_time_s_){
            stair_pause_complete_ = true;
          } else {
            ROS_INFO_THROTTLE(1.0, "Pausing for map stability...");
          }
          if(stair_pause_complete_){
            if(path_gp_dist <= 2.0){
              ROS_INFO("Requesting new path...");
              pub_planning_task_.publish(planning_task_);
              state_ = motionCommandFilter::IDLE;
              have_new_path_ = false;
            } else {
              state_ = motionCommandFilter::PATH_FOLLOW;
            }
          }


        }

        // If the path changes to going down the stairs, just follow it back down
        // For now, we want spot to just send it up the stairs.
        // if(!isUpstairs(path_goal_point_) && false){
        //   state_ = motionCommandFilter::STAIR_MODE_DOWN;
        // }
      }

      break;

    case motionCommandFilter::STAIR_MODE_DOWN:
      // Need to go down the stairs
      if(do_stair_turnaround_){
        float relative_lookahead_heading = atan2((path_lookahead_.y - current_pos_.y), (path_lookahead_.x - current_pos_.x));
        align_heading_error_ = abs(wrapAngle(relative_lookahead_heading - (current_heading_ + M_PI)));

        if(align_heading_error_ < stair_turnaround_thresh_){
          do_stair_turnaround_ = false;
        }
      } else {

        if(checkStairProgress()){
          ROS_INFO("BOTTOM OF STAIRS REACHED");

          // We need to pause for a little bit so we don't go right back down the stairs
          if(!end_stair_pause_){
            end_stair_pause_ = true;
            end_stair_time_ = ros::Time::now();
          }

          float stair_pause_time_diff_s = (ros::Time::now() - end_stair_time_).toSec();
          if(stair_pause_time_diff_s > stair_pause_time_s_){
            stair_pause_complete_ = true;
          }
          if(stair_pause_complete_){
            // If linear distance between goal point and last lookahead is less than some threshold
            if(dist3D(path_goal_point_, path_lookahead_) > .25){
              state_ = motionCommandFilter::PATH_FOLLOW;
            } else {
              state_ = motionCommandFilter::IDLE;
              have_new_path_ = false;
            }
          }

        }
	    }

        // If the path changes to going up the stairs, just follow it up
        // if(isUpstairs(path_goal_point_)){
        //   state_ = motionCommandFilter::STAIR_MODE_UP;
        // }
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

  if(beacon_drop_cmd_ && !is_spot_ && !(state_ == motionCommandFilter::BEACON_DROP || state_ == motionCommandFilter::BEACON_MOTION)){
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



    case motionCommandFilter::STAIR_MODE_UP:
      if(do_stair_align_){
        ROS_INFO_THROTTLE(5.0, "Motion filter: stair mode up, aligning...");

        float relative_lookahead_heading = atan2((path_lookahead_.y - current_pos_.y), (path_lookahead_.x - current_pos_.x));
        align_heading_error_ = wrapAngle(relative_lookahead_heading - current_heading_ );

        float yawrate_cmd = sat(yawrate_k0_*align_heading_error_, -yawrate_max_/2.0, yawrate_max_/2.0);

        control_command_msg_.linear.x = 0.0;
        control_command_msg_.angular.z = yawrate_cmd;
      } else {
        ROS_INFO_THROTTLE(5.0, "Motion filter: stair mode up");

        control_command_msg_.linear.x = stair_mode_fwd_speed_;
        control_command_msg_.angular.z = 0.0;
      }
      break;

    case motionCommandFilter::STAIR_MODE_DOWN:
      if(do_stair_turnaround_){
        ROS_INFO_THROTTLE(5.0, "Motion filter: stair mode down, aligning...");

        float relative_lookahead_heading = atan2((path_lookahead_.y - current_pos_.y), (path_lookahead_.x - current_pos_.x));
        align_heading_error_ = wrapAngle(relative_lookahead_heading - (current_heading_ + M_PI));

        float yawrate_cmd = sat(yawrate_k0_*align_heading_error_, -yawrate_max_/2.0, yawrate_max_/2.0);

        control_command_msg_.linear.x = 0.0;
        control_command_msg_.angular.z = yawrate_cmd;
      } else {
        ROS_INFO_THROTTLE(5.0, "Motion filter: stair mode down");

        control_command_msg_.linear.x = -stair_mode_fwd_speed_;
        control_command_msg_.angular.z = 0.0;
      }
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

  } // end switch case

  // Handle spot stair mode off when not in stairs up / down mode
  if(is_spot_ && is_stair_mode_on_ && !stair_mode_cmd_ && !(state_ == motionCommandFilter::STAIR_MODE_UP || state_ == motionCommandFilter::STAIR_MODE_DOWN)){
    stair_mode_srv_.request.data = false;
    if(stair_mode_client_.call(stair_mode_srv_) || offline_test_){
    //if(true){
      is_stair_mode_on_ = false;
      ROS_INFO("Motion filter: Spot stair mode disengaged.");
    } else {
      ROS_INFO_THROTTLE(1.0, "Motion filter: Spot stair mode disengage failed...");
    }
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

bool motionCommandFilter::isUpstairs(const geometry_msgs::Point lookahead_point){
  float val1 = current_pos_.z + planning_link_z_offset_;
  float z_diff = lookahead_point.z - val1;
  ROS_INFO_THROTTLE(1.0,"z_diff: %f", z_diff);
  //ROS_INFO("z_diff: %f", z_diff);
  if(z_diff > stair_goal_point_offset_){
    return true;
  } else {
    return false;
  }
}

bool motionCommandFilter::isDownstairs(const geometry_msgs::Point lookahead_point){
  float val1 = current_pos_.z + planning_link_z_offset_;
  float z_diff = lookahead_point.z - val1;
  //ROS_INFO("val1: %f, val2: %f, z_diff: %f", val1, lookahead_point.z, z_diff);
  if(z_diff < -stair_goal_point_offset_){
    return true;
  } else {
    return false;
  }
}

bool motionCommandFilter::checkStairProgress(){
  if(!started_stairs_){
    float z_diff = current_pos_.z - start_stair_z_;
    if(abs(current_pitch_) > stair_mode_pitch_thresh_ && abs(z_diff) > z_diff_thresh_){
      started_stairs_ = true;
      ROS_INFO("STARTED STAIRS!!");
    }
    return false;
  } else {
    if(abs(current_pitch_) < stair_mode_pitch_thresh_){
      return true;
    } else {
      return false;
    }
  }

}


float motionCommandFilter::dist(const geometry_msgs::Point p1, const geometry_msgs::Point p2)
{
    float distance = sqrt(pow((p1.x - p2.x),2) + pow((p1.y - p2.y),2));
    return distance;
}

float motionCommandFilter::dist3D(const geometry_msgs::Point p1, const geometry_msgs::Point p2)
{
    float distance = sqrt(pow((p1.x - p2.x),2) + pow((p1.y - p2.y),2) + pow((p1.z - p2.z),2));
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

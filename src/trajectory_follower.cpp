#include <marble_guidance/trajectory_follower.h>

using namespace std;
namespace trajectory_follower{
trajectoryFollower::trajectoryFollower(const ros::NodeHandle &node_handle,
                                       const ros::NodeHandle &private_node_handle)
      :nh_(node_handle),
       pnh_(private_node_handle){
      this->init();
  }

void trajectoryFollower::init() {

    sub_odom_ = nh_.subscribe("odometry_map", 1, &trajectoryFollower::odomCb, this);
    sub_carto_traj_ = nh_.subscribe("cartographer_trajectory", 1, &trajectoryFollower::cartoTrajCb, this);
    sub_liosam_traj_ = nh_.subscribe("liosam_trajectory", 1, &trajectoryFollower::liosamTrajCb, this);
    sub_gt_traj_ = nh_.subscribe("ground_truth_trajectory", 1, &trajectoryFollower::gtTrajCb, this);
    sub_follow_traj_ = nh_.subscribe("follow_traj", 1, &trajectoryFollower::followTrajCb, this);

    pub_lookahead_ = nh_.advertise<geometry_msgs::PointStamped>("lookahead_point", 10);
    pub_traj_motion_cmd_ = nh_.advertise<marble_guidance::MotionCmd>("motion_cmd", 10);
    pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    std::string vehicle_name;
    pnh_.param<std::string>("vehicle_name", vehicle_name,"X1");
    pnh_.param("enable_ground_truth", enable_ground_truth_, false);

    pnh_.param("turn_in_place_thresh", turn_in_place_thresh_, 1.0);
    pnh_.param("turn_in_place_yawrate", turn_in_place_yawrate_, 1.0);
    pnh_.param("yawrate_k0", yawrate_k0_, 1.0);
    pnh_.param("yawrate_kd", yawrate_kd_ , 1.0);
    pnh_.param("yawrate_max", yawrate_max_ , 1.0);
    pnh_.param("lookahead_distance_threshold", lookahead_dist_thresh_, 1.0);
    pnh_.param("max_forward_speed", u_cmd_max_, 1.0);

    last_lookahead_index_ = 0;
    lookahead_dist_short_ = .5;
    lookahead_dist_long_ = 2.0;
    enable_lookahead_lookup_ = false;
    have_current_traj_home_ = false;
    have_current_gt_traj_home_ = false;
    last_gt_lookahead_index_ = 0;
    have_traj_ = false;
    have_odom_ = false;

    string lookahead_frame = vehicle_name + "/map";
    lookahead_point_.header.frame_id = lookahead_frame;

}

void trajectoryFollower::odomCb(const nav_msgs::OdometryConstPtr& odom_msg)
{
    //ROS_INFO_THROTTLE(1,"Received odom");
    if(!have_odom_) have_odom_ = true;
    odom_ = *odom_msg;
    odom_point_ =  odom_.pose.pose.position;
    geometry_msgs::Quaternion vehicle_quat_msg = odom_.pose.pose.orientation;
    tf::Quaternion vehicle_quat_tf;
    tf::quaternionMsgToTF(vehicle_quat_msg, vehicle_quat_tf);
    tf::Matrix3x3(vehicle_quat_tf).getRPY(current_roll_, current_pitch_, current_heading_);
    //ROS_INFO_THROTTLE(1,"x: %f, y: %f", odom_point_.x, odom_point_.y);
}

void trajectoryFollower::cartoTrajCb(const lcd_pkg::PoseGraphConstPtr& msg)
{

    if(!have_traj_) have_traj_ = true;

    //ROS_INFO_THROTTLE(1,"Received traj");
    // e only need to do this when we can't plan home
    uint32_t traj_list_size_ = msg->poseArray.size();

    //ROS_INFO_THROTTLE(1, "Traj list size: %d,", traj_list_size_);
    if(enable_lookahead_lookup_ && !have_current_traj_home_){
        traj_list_points_.clear();
        //ROS_INFO_THROTTLE(1, "traj_list_size: %d", traj_list_size_);
        last_lookahead_index_ = traj_list_size_-1;
        // Import trajectory list
        for (int i = 0; i < traj_list_size_; i++){
            traj_list_points_.push_back(msg->poseArray[i].pose.position);
        }
        //ROS_INFO("%d", last_lookahead_index_);
        have_current_traj_home_ = true;
      }
}

void trajectoryFollower::liosamTrajCb(const nav_msgs::PathConstPtr& msg)
{

    if(!have_traj_) have_traj_ = true;

    //ROS_INFO_THROTTLE(1,"Received traj");
    // e only need to do this when we can't plan home
    uint32_t traj_list_size_ = msg->poses.size();

    //ROS_INFO_THROTTLE(1, "Traj list size: %d,", traj_list_size_);
    if(enable_lookahead_lookup_ && !have_current_traj_home_){
        traj_list_points_.clear();
        //ROS_INFO_THROTTLE(1, "traj_list_size: %d", traj_list_size_);
        last_lookahead_index_ = traj_list_size_-1;
        // Import trajectory list
        for (int i = 0; i < traj_list_size_; i++){
            traj_list_points_.push_back(msg->poses[i].pose.position);
        }
        ROS_INFO("%d", last_lookahead_index_);
        have_current_traj_home_ = true;
      }
}

void trajectoryFollower::gtTrajCb(const marble_guidance::TrajListConstPtr& msg)
{
    //ROS_INFO_THROTTLE(1,"Received gt traj");
    // e only need to do this when we can't plan home

    if(!have_traj_) have_traj_ = true;

    uint32_t gt_traj_list_size_ = msg->traj_list_size;

    //ROS_INFO_THROTTLE(1, "Traj list size: %d,", traj_list_size_);
    if(enable_lookahead_lookup_ && !have_current_gt_traj_home_){
        traj_list_points_.clear();
        //ROS_INFO_THROTTLE(1, "traj_list_size: %d", traj_list_size_);
        last_gt_lookahead_index_ = gt_traj_list_size_-1;
        // Import trajectory list

        gt_traj_list_points_ = msg->traj_points;

        //ROS_INFO("%d", last_lookahead_index_);
        have_current_gt_traj_home_ = true;
      }
}

void trajectoryFollower::findNextLookahead(){
    // Parse through the list for the next lookahead
    //ROS_INFO_THROTTLE(1,"last_lookahead_index: %d", last_lookahead_index_);
    if(have_current_traj_home_ && (last_lookahead_index_ != 0)){
        //ROS_INFO_THROTTLE(1,"last_lookahead_index #2: %d", last_lookahead_index_);
        for (int i = last_lookahead_index_; i > 0; i--){
            float dist_err = dist(odom_point_, traj_list_points_[i]);
            //sROS_INFO("%f, %d", dist_err, i);
            if((dist_err > lookahead_dist_short_) && (dist_err < lookahead_dist_long_)){
                lookahead_point_.point = traj_list_points_[i];
                last_lookahead_index_ = i + 1;
                //ROS_INFO_THROTTLE(1,"last_lookahead_index: %d, x: %f, y: %f", last_lookahead_index_, lookahead_point_.point.x, lookahead_point_.point.y);
                //ROS_INFO_THROTTLE(1,"last_lookahead_index: %d, x: %f, y: %f", last_lookahead_index_, traj_list_points_[i].x, traj_list_points_[i].y);
                //break;
            }

        }
        //ROS_INFO_THROTTLE(1,"last_lookahead_index: %d, x: %f, y: %f", last_lookahead_index_, traj_list_points_[last_lookahead_index_].x, traj_list_points_[last_lookahead_index_].y);
    } else {
        lookahead_point_.point = odom_point_;
    }
}

void trajectoryFollower::findNextGTLookahead(){
    // Parse through the list for the next lookahead
    //ROS_INFO_THROTTLE(1,"last_gt_lookahead_index: %d", last_gt_lookahead_index_);
    if(have_current_gt_traj_home_ && (last_gt_lookahead_index_ != 0)){
        //ROS_INFO('Test1');
        //ROS_INFO_THROTTLE(1,"last_lookahead_index #2: %d", last_lookahead_index_);
        for (int i = last_gt_lookahead_index_; i > 0; i--){
            //ROS_INFO('Test2');

            float dist_err = dist(odom_point_, gt_traj_list_points_[i]);
            //ROS_INFO('Test3');

          //  ROS_INFO("%f, %d", dist_err, i);
            if((dist_err > lookahead_dist_short_) && (dist_err < lookahead_dist_long_)){
                lookahead_point_.point = gt_traj_list_points_[i];
                last_gt_lookahead_index_ = i + 1;
                //ROS_INFO_THROTTLE(1,"last_lookahead_index: %d, x: %f, y: %f", last_lookahead_index_, lookahead_point_.point.x, lookahead_point_.point.y);
                //ROS_INFO_THROTTLE(1,"last_lookahead_index: %d, x: %f, y: %f", last_lookahead_index_, traj_list_points_[i].x, traj_list_points_[i].y);
                //break;
            }

        }
        //ROS_INFO_THROTTLE(1,"last_lookahead_index: %d, x: %f, y: %f", last_gt_lookahead_index_, gt_traj_list_points_[last_gt_lookahead_index_].x, gt_traj_list_points_[last_gt_lookahead_index_].y);
    } else {
        lookahead_point_.point = odom_point_;
    }
}

void trajectoryFollower::publishLookahead(){
    //ROS_INFO_THROTTLE(2,"Publishing trajectory lookahead...");
    if(!enable_lookahead_lookup_){
      lookahead_point_.point = odom_point_;
    }
    lookahead_point_.header.stamp = ros::Time::now();
    pub_lookahead_.publish(lookahead_point_);
}

void trajectoryFollower::computeCmdVel(){
  // Take the lookahead point and compute a forward velocity and steering rate

  if(!enable_lookahead_lookup_){
    u_cmd_ = 0.0;
    yawrate_cmd_ = 0.0;
    turn_in_place_ = false;
  } else {
    // Create a yaw rate command from the heading error to the lookahead point
    float relative_lookahead_heading = atan2((lookahead_point_.point.y - odom_point_.y), (lookahead_point_.point.x - odom_point_.x));
    float lookahead_angle_error = wrapAngle(relative_lookahead_heading - current_heading_);
    float distance = dist(odom_point_, lookahead_point_.point);
    //ROS_INFO_THROTTLE(1, "Rel. Heading: %f, Cur. Heading: %f, Angle Err: %f, Dist: %f", relative_lookahead_heading, current_heading_, lookahead_angle_error, dist);

    if(abs(lookahead_angle_error) < turn_in_place_thresh_){
      // Use an exponential attractor to generate yawrate cmds
      yawrate_cmd_ = sat(yawrate_k0_*lookahead_angle_error*exp(-yawrate_kd_*distance), -yawrate_max_, yawrate_max_);
      turn_in_place_ = false;
    } else {
      // Lookahead angle error is larger than threshold, so we should turn in place
      yawrate_cmd_ = turn_in_place_yawrate_;
      turn_in_place_ = true;
    }

    // FORWARD SPEED COMMAND
    if(!turn_in_place_){
      // Slow down if we are approaching the lookahead point
      u_cmd_ = sat(u_cmd_max_*(1 - ((lookahead_dist_thresh_ -  distance)/lookahead_dist_thresh_)), 0.0, u_cmd_max_);
    } else {
      u_cmd_ = 0.0;
    }
  }

}

void trajectoryFollower::publishMotionCmd(){

  cmd_vel_msg_.linear.x = u_cmd_;
  cmd_vel_msg_.angular.z = yawrate_cmd_;
  pub_cmd_vel_.publish(cmd_vel_msg_);

  // Determine motion type
  // 0: Forward, 1: Turn in place
  if(turn_in_place_){
    traj_motion_cmd_.motion_type = 1;
  } else {
    traj_motion_cmd_.motion_type = 0;
  }

  // Add control commands
  traj_motion_cmd_.cmd_vel.linear.x = u_cmd_;
  traj_motion_cmd_.cmd_vel.angular.z = yawrate_cmd_;

  // Include lookahead point
  traj_motion_cmd_.lookahead_point = lookahead_point_.point;

  pub_traj_motion_cmd_.publish(traj_motion_cmd_);

}

bool trajectoryFollower::getGroundTruth(){
  return enable_ground_truth_;
}

void trajectoryFollower::followTrajCb(const std_msgs::BoolConstPtr& follow_traj_msg){
    //ROS_INFO("Follow traj cb");
    if(follow_traj_msg->data){
        enable_lookahead_lookup_ = true;
    } else {
      enable_lookahead_lookup_ = false;
      last_lookahead_index_ = 0;
      have_current_traj_home_ = false;
      have_current_gt_traj_home_ = false;
      last_gt_lookahead_index_ = 0;
   }
}

float trajectoryFollower::dist(const geometry_msgs::Point p1, const geometry_msgs::Point p2)
{
    float distance = sqrt(pow((p1.x - p2.x),2) + pow((p1.y - p2.y),2));
    return distance;
}

float trajectoryFollower::wrapAngle(float angle){

    if (angle > M_PI){
        angle -= 2*M_PI;
    } else if( angle < -M_PI){
        angle += 2*M_PI;
    }
    return angle;
}

float trajectoryFollower::sat(float num, float min_val, float max_val){

    if (num >= max_val){
        return max_val;
    } else if( num <= min_val){
         return min_val;
    } else {
      return num;
    }

}

bool trajectoryFollower::doLookup(){
    return (enable_lookahead_lookup_);
}



 // end of class
} // End of namespace nearness

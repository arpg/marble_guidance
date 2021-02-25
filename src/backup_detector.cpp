#include <marble_guidance/backup_detector.h>

namespace backup_detector{
backupDetector::backupDetector(const ros::NodeHandle &node_handle,
                                       const ros::NodeHandle &private_node_handle)
      :nh_(node_handle),
       pnh_(private_node_handle) {
      this->init();
  }

void backupDetector::init() {

  sub_laserscan_ = nh_.subscribe("laserscan", 1, &backupDetector::laserscanCb, this);
  sub_octomap_ = nh_.subscribe("octomap", 1, &backupDetector::octomapCb, this);
  sub_imu_ = nh_.subscribe("imu", 1, &backupDetector::imuCb, this);

  pub_backup_ = nh_.advertise<std_msgs::Bool>("enable_backup", 1);

  pnh_.param("total_scan_points", num_scan_points_, 100);
  pnh_.param("min_turnaround_distance", min_turnaround_distance_, 0.1);
  pnh_.param("pitch_limit", pitch_limit_, 20.0);

  backup_msg_.data = false;
  close_obstacle_flag_ = false;
  bad_attitude_flag_ = false;
  have_scan_ = false;
  have_imu_ = false;
  have_octomap_ = false;

  // Initialize the laserscan vector
  laserscan_ranges_.resize(num_scan_points_);

}

void backupDetector::laserscanCb(const sensor_msgs::LaserScanConstPtr& scan_msg)
{
  if(!have_scan_) have_scan_ = true;
  // Import the laserscan
  laserscan_ranges_ = scan_msg->ranges;
}

void octomapCb(const octomap_msgs::Octomap::ConstPtr msg)
{
  if(!have_octomap_) have_octomap_ = true;
  ROS_INFO("Occupancy octomap callback called");
  if (msg->data.size() == 0) return;
  delete occupancyTree;
  occupancyTree = (octomap::OcTree*)octomap_msgs::binaryMsgToMap(*msg);
}

void backupDetector::imuCb(const sensor_msgs::ImuConstPtr& imu_msg){
    if(!have_imu_) have_imu_ = true;
    geometry_msgs::Quaternion vehicle_imu_quat_msg = imu_msg->orientation;
    tf::Quaternion vehicle_imu_quat_tf;
    tf::quaternionMsgToTF(vehicle_imu_quat_msg, vehicle_imu_quat_tf);
    tf::Matrix3x3(vehicle_imu_quat_tf).getRPY(roll_, pitch_, imu_yaw_);
    //roll_ = wrapAngle(roll_ - M_PI);
    //float temp_roll = roll_;
    //roll_ = pitch_;
    //pitch_ = -temp_roll;
}

void backupDetector::processLaserscan(){
  // Check the octomap laserscan to see if we can back up
  // If the minimum value of the scan is less than the vehicle length (min_turnaround_distance_)
  // then the vehicle needs to back up instead of turn around.
  float min_distance = *min_element(laserscan_ranges_.begin(), laserscan_ranges_.end() );

  if (min_distance < min_turnaround_distance_){
    close_obstacle_flag_ = true;
  } else {
    close_obstacle_flag_ = false;
  }

}

void backupDetector::processOctomap(){
  // Check the surrounding octomap to see if we are able to turn around
  // Need to query all voxels in a predefined radius around the vehicle.
  // All must be unoccupied to turn around

  // Generate a list of query coordinates
  // These should be the centers of voxels in question
  for(int i = 0; i < num_query_points; i++){
    octomap::OcTreeKey coord_key;
    coord_key = occupancyTree->coordToKey(x, y, z);
  }

}

void backupDetector::processIMU(){
  // Check to see if the vehicle is pitched past the limit.
  // If so, we need to backup instead of turning around to prevent
  // the vehicle from slipping sideways in sim.
  if(abs(pitch_) > pitch_limit_){
    bad_attitude_flag_ = true;
  } else {
    bad_attitude_flag_ = false;
  }
}

void backupDetector::publishBackupMsg(){
  backup_msg_.data = (close_obstacle_flag_ || bad_attitude_flag_);
  pub_backup_.publish(backup_msg_);
}

bool backupDetector::haveIMU(){
  return have_imu_;
}

bool backupDetector::haveScan(){
  return have_scan_;
}

bool backupDetector::haveOctomap(){
  return have_octomap_;
}


 // end of class
} // End of namespace nearness

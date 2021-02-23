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

  pub_backup_ = nh_.advertise<std_msgs::Bool>("enable_backup", 1);

  pnh_.param("total_scan_points", num_scan_points_, 100);
  pnh_.param("min_turnaround_distance", min_turnaround_distance_, 0.1);

  backup_msg_.data = false;
  have_scan_ = false;

  // Initialize the laserscan vector
  laserscan_ranges_.resize(num_scan_points_);

}

void backupDetector::laserscanCb(const sensor_msgs::LaserScanConstPtr& scan_msg)
{
  // Import the laserscan
  laserscan_ranges_ = scan_msg->ranges;
}

void backupDetector::processLaserscan(){
  // Check the octomap laserscan to see if we can back up
  // If the minimum value of the scan is less than the vehicle length (min_turnaround_distance_)
  // then the vehicle needs to back up instead of turn around.
  float min_distance = *min_element(laserscan_ranges_.begin(), laserscan_ranges_.end() );

  if (min_distance < min_turnaround_distance_){
    backup_msg_.data = true;
  } else {
    backup_msg_.data = false;
  }

  pub_backup_.publish(backup_msg_);
}

bool backupDetector::haveScan(){
  return have_scan_;
}


 // end of class
} // End of namespace nearness

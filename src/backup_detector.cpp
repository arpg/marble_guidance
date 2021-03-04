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
  pub_query_point_pcl_ = nh_.advertise<sensor_msgs::PointCloud2>("query_points", 1);
  pub_transformed_query_point_pcl_ = nh_.advertise<sensor_msgs::PointCloud2>("transformed_query_points", 1);
  pub_occupied_points_pcl_ = nh_.advertise<sensor_msgs::PointCloud2>("occupied_points", 1);

  pnh_.param("total_scan_points", num_scan_points_, 100);
  pnh_.param("min_turnaround_distance", min_turnaround_distance_, 0.1);
  pnh_.param("pitch_limit", pitch_limit_, 20.0);

  // Query point grid parameters
  pnh_.param("num_query_point_rows", num_query_point_rows_, 10);
  pnh_.param("num_query_point_cols", num_query_point_cols_, 10);
  pnh_.param("num_query_point_layers", num_query_point_layers_, 10);
  pnh_.param("query_point_spacing", query_point_spacing_, .2);
  pnh_.param("safety_radius", safety_radius_, .75);
  pnh_.param("z_lower_threshold", z_lower_threshold_, .2);

  pnh_.param("resolution", resolution_, 0.15);

  pnh_.param<std::string>("vehicle_name", vehicle_name_,"X1");
  pnh_.param<std::string>("world_frame", world_frame_, "world");
  pnh_.param<std::string>("base_link_frame", base_link_frame_, "base_link");
  world_frame_ = "simple_cave_01";
  base_link_frame_ = "X1/base_link";



  backup_msg_.data = false;
  enable_debug_ = true;
  close_obstacle_flag_ = false;
  bad_attitude_flag_ = false;
  have_scan_ = false;
  have_imu_ = false;
  have_octomap_ = false;
  num_query_point_rows_ = 10;
  num_query_point_cols_ = 10;
  num_query_point_layers_ = 10;
  num_query_points_ = num_query_point_rows_*num_query_point_cols_*num_query_point_layers_;
  query_point_spacing_ = resolution_;;

  // Merged OcTree
  occupancyTree_ = new octomap::OcTree(resolution_);

  // Initialize the laserscan vector
  laserscan_ranges_.resize(num_scan_points_);

  // Generate the query points for octomap voxel lookups
  generateQueryPoints();

}

void backupDetector::generateQueryPoints(){
  // Generate a simple gridded cube of query points around the vehicle
  float half_width = (num_query_point_rows_*query_point_spacing_)/2.0;
  float half_length = (num_query_point_cols_*query_point_spacing_)/2.0;
  geometry_msgs::Point query_point;
  for(int i = 0; i < num_query_point_rows_; i++){
    for(int j = 0; j < num_query_point_cols_; j++){
      for(int k = 0; k < num_query_point_layers_; k++){
        query_point.x = half_width - i*query_point_spacing_;
        query_point.y = half_length - j*query_point_spacing_;
        query_point.z = k*query_point_spacing_;
        query_point_vec_.push_back(query_point);
      }
    }
  }
}

void backupDetector::publishQueryPointsPcl(){
  // Convert the vector of query points into a pointcloud
  pcl::PointCloud<pcl::PointXYZ> query_point_pcl;
  pcl::PointXYZ pcl_point;
  for (int i = 0; i < num_query_points_; i++){
    pcl_point = {query_point_vec_[i].x, query_point_vec_[i].y, query_point_vec_[i].z};
    query_point_pcl.push_back(pcl_point);
  }

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(query_point_pcl, cloud_msg);
  cloud_msg.header.frame_id = base_link_frame_;
  cloud_msg.header.stamp = ros::Time::now();

  pub_query_point_pcl_.publish(cloud_msg);

  pcl::PointCloud<pcl::PointXYZ> transformed_query_point_pcl;
  int num_transformed_points = transformed_query_point_vec_.size();
  for (int i = 0; i < num_transformed_points; i++){
    pcl_point = {transformed_query_point_vec_[i].point.x, transformed_query_point_vec_[i].point.y, transformed_query_point_vec_[i].point.z};
    transformed_query_point_pcl.push_back(pcl_point);
  }

  pcl::toROSMsg(transformed_query_point_pcl, cloud_msg);
  cloud_msg.header.frame_id = world_frame_;
  cloud_msg.header.stamp = ros::Time::now();

  pub_transformed_query_point_pcl_.publish(cloud_msg);

}

void backupDetector::laserscanCb(const sensor_msgs::LaserScanConstPtr& scan_msg)
{
  if(!have_scan_) have_scan_ = true;
  // Import the laserscan
  laserscan_ranges_ = scan_msg->ranges;
}

void backupDetector::octomapCb(const octomap_msgs::Octomap::ConstPtr msg)
{
  if(!have_octomap_) have_octomap_ = true;
  //ROS_INFO("Occupancy octomap callback called");
  if (msg->data.size() == 0) return;
  delete occupancyTree_;
  occupancyTree_ = (octomap::OcTree*)octomap_msgs::binaryMsgToMap(*msg);
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

void backupDetector::transformQueryPoints(const geometry_msgs::TransformStamped transform_stamped){
  // Transform the set of querypoints from the vehicle frame to the map frame
  geometry_msgs::PointStamped initial_pt;
  initial_pt.header.stamp = ros::Time::now();
  initial_pt.header.frame_id = base_link_frame_;
  geometry_msgs::PointStamped transformed_pt;
  transformed_pt.header.stamp = ros::Time::now();
  transformed_query_point_vec_.clear();
  for(int i= 0; i < num_query_points_; i++){
    initial_pt.point.x = query_point_vec_[i].x;
    initial_pt.point.y = query_point_vec_[i].y;
    initial_pt.point.z = query_point_vec_[i].z;
    tf2::doTransform(initial_pt, transformed_pt, transform_stamped);
    transformed_query_point_vec_.push_back(transformed_pt);
  }

}

void backupDetector::processOctomap(){
  // Check the surrounding octomap to see if we are able to turn around
  // Need to query all voxels in a predefined radius around the vehicle.
  // All must be unoccupied to turn around

  // Generate a list of query coordinates
  // These should be the centers of voxels in question
  // Convert the query coordinates in the map frame to
  // voxel keys in the map frame.
  geometry_msgs::Point qp;
  for(int i = 0; i < num_query_points_; i++){
    qp = transformed_query_point_vec_[i].point;
    coord_key_vec_.push_back(occupancyTree_->coordToKey(qp.x, qp.y, qp.z));
  }

  // Check each keyed voxel for occupancy
  occupied_cell_indices_vec_.clear();
  for(int i = 0; i < num_query_points_; i++){
    octomap::OcTreeNode* current_node = occupancyTree_->search(coord_key_vec_[i]);
    if(current_node && current_node->getLogOdds() >= .5){
      // Cell is occupied, need to track the index
      occupied_cell_indices_vec_.push_back(i);
    }
  }

  // Check all occupied cells to see if they exist
  // within some predefined safety radius around the vehicle
  // Also have a height check for tuning voxels near the wheels
  float voxel_radius;
  close_obstacle_flag_ = false;
  int index;
  for (int i = 0; i < occupied_cell_indices_vec_.size(); i++){
    index = occupied_cell_indices_vec_[i];
    voxel_radius = sqrt(pow(query_point_vec_[index].x, 2) + pow(query_point_vec_[index].y, 2));
    if(voxel_radius < safety_radius_){
      if(query_point_vec_[index].z > z_lower_threshold_){
       close_obstacle_flag_ = true;
      }
    }
  }

  if (enable_debug_){
    pcl::PointCloud<pcl::PointXYZ> point_pcl;
    pcl::PointXYZ pcl_point;
    for (int i = 0; i < occupied_cell_indices_vec_.size(); i++){
      index = occupied_cell_indices_vec_[i];
      pcl_point = {transformed_query_point_vec_[index].point.x, transformed_query_point_vec_[index].point.y, transformed_query_point_vec_[index].point.z};
      point_pcl.push_back(pcl_point);
    }

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(point_pcl, cloud_msg);
    cloud_msg.header.frame_id = world_frame_;
    cloud_msg.header.stamp = ros::Time::now();

    pub_occupied_points_pcl_.publish(cloud_msg);
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

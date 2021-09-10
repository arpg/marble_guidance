#include <marble_guidance/husky_safety.h>

using namespace std;
namespace husky_safety{
huskySafety::huskySafety(const ros::NodeHandle &node_handle,
                                       const ros::NodeHandle &private_node_handle)
      :nh_(node_handle),
       pnh_(private_node_handle){
      this->init();
  }

void huskySafety::init() {

  // sub_odom_ = nh_.subscribe("odometry_map", 1, &huskySafety::odomCb, this);
  sub_rplidar_ = nh_.subscribe("rplidar_scan", 1, &huskySafety::rplidarScanCb, this);

  // pub_scan_final_ = nh_.advertise<std_msgs::Float32MultiArray>("scan_final", 1);
  pub_scan_final_ = nh_.advertise<sensor_msgs::LaserScan>("scan_final", 1);
  pub_recon_wf_nearness_ = nh_.advertise<std_msgs::Float32MultiArray>("recon_wf_nearness", 1);
  pub_sf_nearness_ = nh_.advertise<std_msgs::Float32MultiArray>("sf_nearness", 1);
  pub_sf_nearness_cmd_ = nh_.advertise<std_msgs::Float32>("sf_nearness_cmd", 1);
  pub_safety_status_ = nh_.advertise<marble_guidance::HuskySafety>("safety_status",1);
  // pub_cmd_vel_stamped_ = nh_.advertise<geometry_msgs::TwistStamped>("cmd_vel_stamped", 10);
  //pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  pnh_.param<std::string>("vehicle_name", vehicle_name_,"X1");
  pnh_.param("total_scan_points", total_scan_points_, 720);
  pnh_.param("num_scan_points", num_scan_points_, 360);
  pnh_.param("scan_start_index", scan_start_index_, 180);
  pnh_.param("scan_frequency", scan_freq_, 10.0);
  pnh_.param("scan_limit", scan_limit_, M_PI/2.0);
  pnh_.param("total_fourier_terms", total_fourier_terms_, 10);
  pnh_.param("num_small_field_fourier_terms", num_sf_fourier_terms_, 5);
  pnh_.param("small_field_threshold_gain", sf_thresh_k_, 3.0);

  pnh_.param("small_field_yawrate_gain", sf_k_0_, 1.0);
  pnh_.param("small_field_psi_gain", sf_k_psi_, 1.0);
  pnh_.param("small_field_distance_gain", sf_k_d_, 1.0);

  pnh_.param("front_safety_distance", f_dist_, 0.25);
  pnh_.param("side_safety_distance", s_dist_, 0.35);
  pnh_.param("enable_sf_control", enable_sf_control_, true);

  have_scan_ = false;
  debug_ = false;

  max_sensor_dist_ = 40.0;
  min_dist_ = max_sensor_dist_;

  generateProjectionShapes();
  generateSafetyBoundary();

}

void huskySafety::generateSafetyBoundary(){
  ROS_INFO("Generating safety box.");
  bool safety_box_corner_switch = false;
  // Generate the left boundary
  for(int i = 0; i < num_scan_points_/2; i++){
    if(dg_*i < atan(f_dist_/s_dist_)){
      safety_boundary_.push_back(s_dist_/cos(dg_*float(i)));
    } else {
      if(!safety_box_corner_switch){
          safety_box_corner_switch = true;
          left_corner_index_ = i;
      }
      safety_boundary_.push_back(f_dist_/cos(M_PI/2 - dg_*float(i)));
    }
  }
  // Copy the left side to the right
  for(int i = 0; i < num_scan_points_/2; i++){
      safety_boundary_.push_back(safety_boundary_[num_scan_points_/2 - i]);
  }
  safety_boundary_[num_scan_points_/2] = f_dist_;
}


void huskySafety::rplidarScanCb(const sensor_msgs::LaserScan::ConstPtr& scan_msg){
  if (!have_scan_) {
    have_scan_ = true;
    scan_final_msg_ = *scan_msg;
  }

  scan_ranges_.clear();
  scan_ranges_ = scan_msg->ranges;
  // for (int i = total_scan_points_; i < total_scan_points_; i++){
  //   scan_ranges_.push_back(scan_msg->ranges[i]);
  // }
  last_scan_time_ = ros::Time::now();
}

void huskySafety::processLidarScan(){
  // Check to make sure we are still getting scans
  float scan_time_diff_s = (ros::Time::now() - last_scan_time_).toSec();
  if (scan_time_diff_s > 10.0*(1.0/scan_freq_)){
    have_scan_ = false;
  }

  // Handle infs
  // IMPLEMENT FROM NEARNESS CONTROLLER IF NEEDED

  // Reverse the scan
  reverse(scan_ranges_.begin(), scan_ranges_.end());

  // Reformat the depth scan depending on the orientation of the scanner
  // This code handles the case where the rplidar case points in the
  // -X direction
  scan_ranges_reformat_.clear();
  for (int i = total_scan_points_/2; i < total_scan_points_; i++) {
    if(isinf(scan_ranges_[i])){
      scan_ranges_reformat_.push_back(max_sensor_dist_);
    } else{
      scan_ranges_reformat_.push_back(scan_ranges_[i]);
    }
  }
  for (int i = 0; i < total_scan_points_/2; i++){
    if(isinf(scan_ranges_[i])){
      scan_ranges_reformat_.push_back(max_sensor_dist_);
    } else{
      scan_ranges_reformat_.push_back(scan_ranges_[i]);
    }
  }

  // Trim the scan down if the entire scan is not being used
  scan_ranges_final_.clear();
  for(int i = 0; i < num_scan_points_; i++){
    scan_ranges_final_.push_back(scan_ranges_reformat_[i+scan_start_index_]);
  }

  if(debug_){
      // std_msgs::Float32MultiArray scan_final_msg;
      // scan_final_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
      // scan_final_msg.layout.dim[0].size = scan_ranges_final_.size();
      // scan_final_msg.data.clear();
      // scan_final_msg.data.insert(scan_final_msg.data.end(), scan_ranges_final_.begin(), scan_ranges_final_.end());
      // pub_scan_final_.publish(scan_final_msg);

      //sensor_msgs::LaserScan scan_final_msg;
      scan_final_msg_.header.frame_id = vehicle_name_ + "/rplidar_safety_link";
      scan_final_msg_.ranges = scan_ranges_final_;
      pub_scan_final_.publish(scan_final_msg_);

  }

  // Convert to CVMat type for easier processing
  // Last, convert to cvmat and saturate
  scan_cvmat_ = cv::Mat(1,num_scan_points_, CV_32FC1);
  std::memcpy(scan_cvmat_.data, scan_ranges_final_.data(), scan_ranges_final_.size()*sizeof(float));

  float cos_proj_arr[total_fourier_terms_ + 1][num_scan_points_];
  float sin_proj_arr[total_fourier_terms_ + 1][num_scan_points_];

  cv::Mat cos_proj_mat(total_fourier_terms_+1, num_scan_points_, CV_32FC1, cos_proj_arr);
  cv::Mat sin_proj_mat(total_fourier_terms_+1, num_scan_points_, CV_32FC1, sin_proj_arr);

  for (int i = 0; i < total_fourier_terms_; i++){
    for (int j = 0; j < num_scan_points_; j++){
      cos_proj_arr[i][j] = cos_proj_array_[i][j];
      sin_proj_arr[i][j] = sin_proj_array_[i][j];
    }
  }

  // Extract WF Fourier coefficients from scan
  // Compute nearness
  nearness_ = cv::Mat::zeros(cv::Size(1, num_scan_points_), CV_32FC1);
  nearness_ = 1.0 / scan_cvmat_;

  // Compute the fourier coefficients
  nearness_a_.clear();
  nearness_b_.clear();
  for (int i = 0; i < total_fourier_terms_; i++){
    nearness_a_.push_back( nearness_.dot(cos_proj_mat.row(i))*dg_/M_PI );
    nearness_b_.push_back( nearness_.dot(sin_proj_mat.row(i))*dg_/M_PI );
  }


} // end of processLidarScan

void huskySafety::generateProjectionShapes(){

  for(int i = 0; i < num_scan_points_; i++){
      gamma_vector_.push_back( i /float(num_scan_points_) * (2.0*scan_limit_) - scan_limit_);
  }
  dg_ = (2.0*scan_limit_)/num_scan_points_;

  vector<float> cos_proj;
  vector<float> sin_proj;

  for (int i = 0; i < total_fourier_terms_ + 1; i++) {
    cos_proj.clear();
    sin_proj.clear();

    for (int j = 0; j < num_scan_points_; j++) {
        cos_proj.push_back(cos(float(i) * gamma_vector_[j]));
        sin_proj.push_back(sin(float(i) * gamma_vector_[j]));
    }
     cos_proj_array_.push_back(cos_proj);
     sin_proj_array_.push_back(sin_proj);
  }

} // end generateProjectionShapes

void huskySafety::computeSFCommands(){

  std::vector<float> recon_wf_nearness(num_scan_points_, 0.0);

  // Reconstruct the WF signal
  for(int i = 0; i < num_scan_points_; i++){
    for(int n = 0; n < num_sf_fourier_terms_; n++){
      recon_wf_nearness[i] += nearness_a_[n+1]*cos((n+1)*gamma_vector_[i]) + nearness_b_[n+1]*sin((n+1)*gamma_vector_[i]);
    }
    recon_wf_nearness[i] += nearness_a_[0]/2.0;
  }

  // Remove the reoconstructed WF signal from the measured nearness signal
  float sf_mean_sum = 0.0;
  float sf_mean_val = 0.0;
  std::vector<float> sf_nearness(num_scan_points_, 0.0);
  std::vector<float> nearness(nearness_.begin<float>(), nearness_.end<float>());

  for(int i=0; i < num_scan_points_; i++){
    sf_nearness[i] = abs(nearness[i] - recon_wf_nearness[i]);
  }

  // Band-aid for bad sf edges
  // Trim the ends off the sf_signal
  int edge_band = 35;
  std::vector<float> sf_nearness_trimmed;
  for (int i = edge_band; i < (num_scan_points_ - edge_band); i++){
    sf_nearness_trimmed.push_back(sf_nearness[i]);
  }
  int num_trimmed_points = num_scan_points_-(2*edge_band);

  // Compute the standard deviation of the SF signal
  sf_mean_val = sf_mean_sum / float(num_trimmed_points);
  float sf_std_dev = 0.0;
  for (int i= 0; i < num_trimmed_points; i++){
      sf_std_dev += pow((sf_nearness_trimmed[i] - sf_mean_val), 2);
  }
  sf_std_dev = pow(sf_std_dev / num_trimmed_points, .5);
  float sf_min_threshold = sf_thresh_k_ * sf_std_dev;

  // Do clustering and mixing
  std::vector<float> sf_d_cluster;
  std::vector<float> sf_r_cluster;
  std::vector<float> cluster_d(200, 0.0);
  std::vector<float> cluster_r(200, 0.0);
  int n = 0;
  int c = 0;
  num_sf_clusters_ = 0;
  //h_sf_min_threshold = .5;
  for(int i=0; i < num_trimmed_points; i++){
    if((sf_nearness_trimmed[i] > sf_min_threshold) && (sf_nearness_trimmed[i+1] > sf_min_threshold)){
      sf_d_cluster.push_back(sf_nearness_trimmed[i]);
      sf_r_cluster.push_back(gamma_vector_[i+edge_band]);
      n++;
    } else {
      if (n > 0){
        for(int j = 0; j < n; j++){
          cluster_d[c] += sf_d_cluster[j];
          cluster_r[c] += sf_r_cluster[j];
        }
        sf_d_cluster.clear();
        sf_r_cluster.clear();
        cluster_d[c] /= float(n);
        cluster_r[c] /= float(n);
        c++;
        n = 0;
      }
    }
  }

  // Compute the SF yawrate command from SF clusters
  num_sf_clusters_ = c;
  sf_r_cmd_ = 0.0;
  int sign = 1;
  if(num_sf_clusters_ != 0){
    for(int i = 0; i < num_sf_clusters_; i++){
      if(cluster_r[i] < 0) sign = 1;
      if(cluster_r[i] > 0) sign = -1;
      sf_r_cmd_ += -sf_k_0_*float(sign)*exp(-sf_k_psi_*abs(cluster_r[i]))*exp(-sf_k_d_/abs(cluster_d[i]));
    }
  }


  // Publish sf nearness signal
  if(debug_){


    std_msgs::Float32MultiArray sf_nearness_msg;
    sf_nearness_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    sf_nearness_msg.layout.dim[0].size = sf_nearness.size();
    sf_nearness_msg.data.clear();
    sf_nearness_msg.data.insert(sf_nearness_msg.data.end(), sf_nearness_trimmed.begin(), sf_nearness_trimmed.end());
    pub_sf_nearness_.publish(sf_nearness_msg);

    std_msgs::Float32MultiArray recon_wf_nearness_msg;
    recon_wf_nearness_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    recon_wf_nearness_msg.layout.dim[0].size = recon_wf_nearness.size();
    recon_wf_nearness_msg.data.clear();
    recon_wf_nearness_msg.data.insert(recon_wf_nearness_msg.data.end(), recon_wf_nearness.begin(), recon_wf_nearness.end());
    pub_recon_wf_nearness_.publish(recon_wf_nearness_msg);

  }

} // end of computeSFCommands

bool huskySafety::haveScan(){
  return have_scan_;
}

void huskySafety::determineSafetyState(){

  // Check the safety box to see if the RPLidar detects any close obstacles
  if(have_scan_){
    checkSafetyBoundary(scan_ranges_final_);
  } else {
    // We should probably stop because a critical sensor went down
    too_close_front_ = false;
    too_close_side_ = false;
  }

  if(enable_sf_control_){
    std_msgs::Float32 sf_nearness_cmd_msg;
    sf_nearness_cmd_msg.data = sf_r_cmd_;
    pub_sf_nearness_cmd_.publish(sf_nearness_cmd_msg);
  }

  husky_safety_msg_.too_close_side = too_close_side_;
  husky_safety_msg_.too_close_front = too_close_front_;
  husky_safety_msg_.min_dist = min_dist_;

  pub_safety_status_.publish(husky_safety_msg_);

}

void huskySafety::checkSafetyBoundary(std::vector<float> scan){
  too_close_front_ = false;
  too_close_side_ = false;

  float min_dist_thresh = 1.0;
  min_dist_ = max_sensor_dist_;
  for(int i = 0; i < num_scan_points_; i++){
    // if((scan[i] < safety_boundary_[i]) && (scan[i] > h_sensor_min_noise_)){
      if(scan[i] < min_dist_thresh){
        min_dist_= scan[i];
      }
      if((scan[i] < safety_boundary_[i])){
        if((i <= left_corner_index_) || (i >= (num_scan_points_ - left_corner_index_))) {
          too_close_side_ = true;
          //ROS_INFO("TOO CLOSE SIDE: index: %d, boundary: %f, val: %f", i, safety_boundary_[i], scan[i]);
        } else {
          too_close_front_ = true;
          //ROS_INFO_THROTTLE(1.0,"TOO CLOSE FRONT");
        }
    } else {
      too_close_front_ = too_close_front_ || too_close_front_;
      too_close_side_ = too_close_side_ || too_close_side_;
    }
  }
}



float huskySafety::dist(const geometry_msgs::Point p1, const geometry_msgs::Point p2)
{
    float distance = sqrt(pow((p1.x - p2.x),2) + pow((p1.y - p2.y),2));
    return distance;
}

float huskySafety::wrapAngle(float angle){

    if (angle > M_PI){
        angle -= 2*M_PI;
    } else if( angle < -M_PI){
        angle += 2*M_PI;
    }
    return angle;
}

float huskySafety::sat(float num, float min_val, float max_val){

    if (num >= max_val){
        return max_val;
    } else if( num <= min_val){
         return min_val;
    } else {
      return num;
    }
}

 // end of class
} // End of namespace husky_safety

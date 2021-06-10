#ifndef BACKUP_DETECTOR_H
#define BACKUP_DETECTOR_H
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <iostream>
#include <random>
#include <vector>

#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/tf.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Octomap libaries
#include <octomap/octomap.h>
//#include <octomap/OcTreeStamped.h>
#include <rough_octomap/RoughOcTree.h>
#include <octomap/OcTreeKey.h>
#include <octomap/ColorOcTree.h>
#include <octomap_msgs/Octomap.h>
// #include <octomap_msgs/conversions.h>
#include <rough_octomap/conversions.h>

using namespace std;

namespace backup_detector{

class backupDetector{
  public:
    backupDetector(const ros::NodeHandle &node_handle,
                            const ros::NodeHandle &private_node_handle);
    ~backupDetector() = default;

    // Functions
    void init();
    bool haveIMU();
    bool haveOctomap();
    void processIMU();
    void processOctomap();
    void publishBackupMsg();
    void publishQueryPointsPcl();
    void generateQueryPoints();
    void transformQueryPoints(const geometry_msgs::TransformStamped tranform_stamped);
    vector<string> getFrames();


  private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    std::string node_name_{"node_name"};

    // Subsribers
    ros::Subscriber sub_octomap_;
    ros::Subscriber sub_imu_;

    // Publishers
    ros::Publisher pub_backup_;
    ros::Publisher pub_query_point_pcl_;
    ros::Publisher pub_transformed_query_point_pcl_;
    ros::Publisher pub_occupied_points_pcl_;
    ros::Publisher pub_close_points_pcl_;

    // Functions
    void octomapCb(const octomap_msgs::Octomap::ConstPtr msg);
    void imuCb(const sensor_msgs::ImuConstPtr& imu_msg);


    bool have_octomap_;
    bool have_imu_;
    bool enable_debug_;

    std::string vehicle_name_;
    std::string map_frame_;
    std::string base_link_frame_;

    std_msgs::Bool backup_msg_;

    double safety_radius_;
    double z_lower_threshold_;

    // Query point parameters
    int num_query_point_rows_;
    int num_query_point_cols_;
    int num_query_point_layers_;
    double query_point_spacing_;
    double num_query_points_;
    vector <geometry_msgs::Point> query_point_vec_;
    vector <geometry_msgs::PointStamped> transformed_query_point_vec_;

    bool close_obstacle_flag_;

    // imuCb
    double roll_;
    double pitch_;
    double imu_yaw_;
    double pitch_limit_;
    bool bad_attitude_flag_;

    // Octomap
    octomap::RoughOcTree* occupancyTree_; // OcTree object for holding occupancy Octomap
    vector<octomap::OcTreeKey> coord_key_vec_;
    vector<int> occupied_cell_indices_vec_;
    vector<int> close_cell_indices_vec_;

    double resolution_;

};

} // namespace

#endif

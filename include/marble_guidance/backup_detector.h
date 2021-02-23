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

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/tf.h>


using namespace std;

namespace backup_detector{

class backupDetector{
  public:
    backupDetector(const ros::NodeHandle &node_handle,
                            const ros::NodeHandle &private_node_handle);
    ~backupDetector() = default;

    // Functions
    void init();
    bool haveScan();
    bool haveIMU();
    void processLaserscan();
    void processIMU();
    void publishBackupMsg();

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::NodeHandle priv_nh;
    std::string node_name_{"node_name"};

    // Subsribers
    ros::Subscriber sub_laserscan_;

    // Publishers
    ros::Publisher pub_backup_;

    // Functions
    void laserscanCb(const sensor_msgs::LaserScanConstPtr& scan_msg);
    void imuCb(const sensor_msgs::ImuConstPtr& imu_msg);

    bool have_scan_;
    bool have_imu_;

    std_msgs::Bool backup_msg_;

    int num_scan_points_;
    double min_turnaround_distance_;
    vector<float> laserscan_ranges_;
    bool close_obstacle_flag_;

    // imuCb
    double roll_;
    double pitch_;
    double imu_yaw_;
    double pitch_limit_;
    bool bad_attitude_flag_;

};

} // namespace

#endif

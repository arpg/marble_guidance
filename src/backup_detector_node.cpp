#include "marble_guidance/backup_detector.h"
#include <string>

int main(int argc, char** argv) {
    std::string node_name = "backup_detector_node";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh("backup_detector");
    ros::NodeHandle nh_private("~");
    backup_detector::backupDetector backup_detector(nh, nh_private);

    backup_detector.generateQueryPoints();

    // Transform buffer
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    geometry_msgs::TransformStamped transform_stamped;

    ros::Rate loop_rate(10);

    while(ros::ok()){
        // Check for a new tf

        try {
          transform_stamped = tf_buffer.lookupTransform("world", "X1/base_link", ros::Time(0));
        } catch (tf2::TransformException &ex) {
          ROS_WARN("%s",ex.what());
          ros::Duration(1.0).sleep();
          continue;
        }

        // if(backup_detector.haveScan()){
        //   backup_detector.processLaserscan();
        // }
        //
        // if(backup_detector.haveIMU()){
        //   backup_detector.processIMU();
        // }
        //

        if(backup_detector.haveOctomap()){
          backup_detector.transformQueryPoints(transform_stamped);
          backup_detector.processOctomap();
        }

        // backup_detector.publishBackupMsg();
        backup_detector.publishQueryPointsPcl();
        ros::spinOnce();
        loop_rate.sleep();
    }

}

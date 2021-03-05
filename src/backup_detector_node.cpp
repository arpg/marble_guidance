#include "marble_guidance/backup_detector.h"
#include <string>

int main(int argc, char** argv) {
    std::string node_name = "backup_detector_node";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh("backup_detector");
    ros::NodeHandle nh_private("~");
    backup_detector::backupDetector backup_detector(nh, nh_private);

    backup_detector.generateQueryPoints();
    vector<string> frames = backup_detector.getFrames();

    // Transform buffer
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    geometry_msgs::TransformStamped transform_stamped;

    ros::Rate loop_rate(10);

    while(ros::ok()){

        // Check for a new tf
        try {
          transform_stamped = tf_buffer.lookupTransform(frames[0], frames[1], ros::Time(0));
        } catch (tf2::TransformException &ex) {
          ROS_WARN("%s",ex.what());
          ros::Duration(1.0).sleep();
          continue;
        }

        // Check if our attitude is outside of the safety bounds
        if(backup_detector.haveIMU()){
          backup_detector.processIMU();
        }

        // Check if we are clear of obstacles
        if(backup_detector.haveOctomap()){
          backup_detector.transformQueryPoints(transform_stamped);
          backup_detector.processOctomap();
        }

        // Publish results
        backup_detector.publishBackupMsg();
        backup_detector.publishQueryPointsPcl();
        ros::spinOnce();
        loop_rate.sleep();
    }

}

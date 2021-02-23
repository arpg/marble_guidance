#include "marble_guidance/backup_detector.h"
#include <string>

int main(int argc, char** argv) {
    std::string node_name = "backup_detector_node";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh("backup_detector");
    ros::NodeHandle nh_private("~");
    backup_detector::backupDetector backup_detector(nh, nh_private);

    ros::Rate loop_rate(1);

    while(ros::ok()){
        if(backup_detector.haveScan()){
          backup_detector.processLaserscan();
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

}

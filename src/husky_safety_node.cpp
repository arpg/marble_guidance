#include "marble_guidance/husky_safety.h"
#include <string>

int main(int argc, char** argv) {
  std::string node_name = "husky_safety";
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("husky_safety");
  ros::NodeHandle nh_private("~");
  //nearness::NearnessController nearness_controller_node(nh, nh_private);
  husky_safety::huskySafety husky_safety_node(nh, nh_private);

  int rate;
  bool enable_sf_control;
  nh_private.param("loop_rate", rate, 10);
  nh_private.param("enable_sf_control", enable_sf_control, false);
  ros::Rate loop_rate(rate);

  while(ros::ok()){

    // Process any new lidar scans
    if(husky_safety_node.haveScan()){
      husky_safety_node.processLidarScan();
      if(enable_sf_control){
        husky_safety_node.computeSFCommands();
      }

    } else {
      ROS_INFO_THROTTLE(1.0, "Do not have any new RPlidar scans.");
    }

    // Determine the safety state
    husky_safety_node.determineSafetyState();

    ros::spinOnce();
    loop_rate.sleep();

  }
}

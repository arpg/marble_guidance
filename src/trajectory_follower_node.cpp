#include "marble_guidance/trajectory_follower.h"
#include <string>

int main(int argc, char** argv) {
  std::string node_name = "trajectory_follower";
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("trajectory_follower");
  ros::NodeHandle nh_private("~");
  //nearness::NearnessController nearness_controller_node(nh, nh_private);
  trajectory_follower::trajectoryFollower trajectory_follower_node(nh, nh_private);

  ros::Rate loop_rate(trajectory_follower_node.getLoopRate());
  bool enable_ground_truth = trajectory_follower_node.getGroundTruth();

  while(ros::ok()){

    if(trajectory_follower_node.doLookup()){
      if(enable_ground_truth){
        trajectory_follower_node.findNextGTLookahead();
      } else {
        trajectory_follower_node.findNextLookahead();
      }
    }

    trajectory_follower_node.publishLookahead();
    trajectory_follower_node.computeCmdVel();
    trajectory_follower_node.publishMotionCmd();

    ros::spinOnce();
    loop_rate.sleep();
  }
}

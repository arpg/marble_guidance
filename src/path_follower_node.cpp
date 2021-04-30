#include "marble_guidance/path_follower.h"
#include <string>

int main(int argc, char** argv) {
    std::string node_name = "path_follower_node";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh("path_follower");
    ros::NodeHandle nh_private("~");
    path_follower::pathFollower path_follower(nh, nh_private);

    int rate;
    nh_private.param("loop_rate", rate, 10);
    ros::Rate loop_rate(rate);

    while(ros::ok()){

        if(path_follower.ready()){
          ROS_INFO_THROTTLE(1,"Running...");
          path_follower.computeControlCommands();
          path_follower.publishMotionCmd();
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

}

#include "marble_guidance/motion_command_filter.h"
#include <string>

int main(int argc, char** argv) {
    std::string node_name = "motion_command_filter";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh("motion_command_filter");
    ros::NodeHandle nh_private("~");
    //nearness::NearnessController nearness_controller_node(nh, nh_private);
    motion_command_filter::motionCommandFilter motion_command_filter_node(nh, nh_private);
    
    int rate;
    nh_private.param("loop_rate", rate, 10);
    ros::Rate loop_rate(rate);
    //ros::Rate loop_rate(motion_command_filter_node.getLoopRate());

    while(ros::ok()){

        motion_command_filter_node.filterCommands();

        ros::spinOnce();
        loop_rate.sleep();

    }
}

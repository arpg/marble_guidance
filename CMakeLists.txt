cmake_minimum_required(VERSION 2.8.3)
project(marble_guidance)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  sensor_msgs
  tf
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)




###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES marble_guidance
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs tf2_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(path_follower src/path_follower.cpp)
#add_dependencies(path_follower ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(path_follower ${catkin_LIBRARIES} )

add_executable(path_follower_node src/path_follower_node.cpp)
target_link_libraries(path_follower_node path_follower ${catkin_LIBRARIES})

add_executable(tf2odom src/tf2odom.cpp)
target_link_libraries(tf2odom ${catkin_LIBRARIES})

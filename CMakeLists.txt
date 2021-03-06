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
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
  pcl_ros
  octomap_ros
  octomap_msgs
  lcd_pkg
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

find_package(octomap REQUIRED)

add_message_files(
   FILES
   MotionCmd.msg
   TrajList.msg
   )

 ## Generate added messages and services with any dependencies listed here
 generate_messages(
    DEPENDENCIES
    sensor_msgs
    geometry_msgs
    std_msgs
 )

###################################
## catkin specific configuration ##
###################################

catkin_package(
#   INCLUDE_DIRS include   ${OCTOMAP_INCLUDE_DIRS}
#  LIBRARIES marble_guidance
#   CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs tf2_ros pcl_ros
   DEPENDS OCTOMAP
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(path_follower src/path_follower.cpp)
#add_dependencies(path_follower ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(path_follower ${catkin_LIBRARIES} )

add_executable(path_follower_node src/path_follower_node.cpp)
target_link_libraries(path_follower_node path_follower ${catkin_LIBRARIES})

add_library(trajectory_follower_node src/trajectory_follower_node.cpp)
target_link_libraries(trajectory_follower_node ${catkin_LIBRARIES} )

add_executable(trajectory_follower src/trajectory_follower.cpp)
target_link_libraries(trajectory_follower trajectory_follower_node ${catkin_LIBRARIES})

add_library(trajectory_generator_node src/trajectory_generator_node.cpp)
target_link_libraries(trajectory_generator_node ${catkin_LIBRARIES} )

add_executable(trajectory_generator src/trajectory_generator.cpp)
target_link_libraries(trajectory_generator trajectory_generator_node ${catkin_LIBRARIES})

add_library(backup_detector src/backup_detector.cpp)
#add_dependencies(backup_detector ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(backup_detector ${catkin_LIBRARIES} )

add_executable(backup_detector_node src/backup_detector_node.cpp)
target_link_libraries(backup_detector_node backup_detector ${catkin_LIBRARIES})

add_library(motion_command_filter src/motion_command_filter.cpp)
target_link_libraries(motion_command_filter ${catkin_LIBRARIES} )

add_executable(motion_command_filter_node src/motion_command_filter_node.cpp)
target_link_libraries(motion_command_filter_node motion_command_filter ${catkin_LIBRARIES})

add_executable(tf2odom src/tf2odom.cpp)
target_link_libraries(tf2odom ${catkin_LIBRARIES})

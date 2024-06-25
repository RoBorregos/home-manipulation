cmake_minimum_required(VERSION 3.0.2)
project(frida_arm_joints_server)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  actionlib
  actionlib_msgs
  geometry_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_perception
  roscpp
  roslib
  rospy
  pcl_ros
  pcl_conversions
  std_msgs
  std_srvs
  tf
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  laser_geometry
  frida_manipulation_interfaces
)

## Generate messages in the 'msg' folder
#add_message_files(
#  FILES
#)

## Generate services in the 'srv' folder
#add_service_files(
#  FILES
#)

## Generate actions in the 'action' folder
#add_action_files(
#  DIRECTORY action
#  FILES
#)

## Generate added messages and services with any dependencies listed here
#generate_messages(
# DEPENDENCIES
#  std_msgs actionlib_msgs geometry_msgs moveit_msgs
#)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
  std_msgs actionlib_msgs
    rospy
    actionlib
    roscpp
    roslib
    moveit_core
    moveit_ros_planning_interface
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

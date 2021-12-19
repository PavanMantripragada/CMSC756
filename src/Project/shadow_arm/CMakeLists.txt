cmake_minimum_required(VERSION 3.0.2)
project(shadow_arm)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  tf2
  tf2_ros
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_core
  moveit_visual_tools
  shadowlibs
  trajectory_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs tf2 tf2_ros moveit_msgs moveit_ros_planning     
  moveit_ros_planning_interface moveit_core moveit_visual_tools shadowlibs trajectory_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(pregrasp_pose_publisher src/pregrasp_pose_publisher.cpp )
target_link_libraries(pregrasp_pose_publisher ${catkin_LIBRARIES})

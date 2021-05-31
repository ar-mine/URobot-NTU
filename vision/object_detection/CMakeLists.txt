cmake_minimum_required(VERSION 3.1.3)
project(object_detection)

find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs
)

add_subdirectory(script)


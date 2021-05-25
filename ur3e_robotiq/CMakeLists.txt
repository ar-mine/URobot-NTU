cmake_minimum_required(VERSION 3.1.3)
project(ur3e_robotiq)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    rospy
    interactive_markers
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_perception
    rviz_visual_tools
    moveit_visual_tools
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    octomap_ros
    octomap_msgs
)
find_package(octomap REQUIRED)
add_definitions(-DOCTOMAP_NODEBUGOUT)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(PCL REQUIRED)
find_package(realsense2 REQUIRED)


catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
  DEPENDS
    EIGEN3
)


include_directories(${THIS_PACKAGE_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS})
include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS} ${OCTOMAP_LIBRARIES})
include_directories(${PCL_INCLUDE_DIRS} ${realsense2_INCLUDE_DIRS})

add_subdirectory(src)

 catkin_install_python(PROGRAMS
   script/gripperSever.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud2.h>

// MoveIt
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>

class moveit_scene
{
private:
  ros::Subscriber sub;
  ros::Publisher pub;
  
public:
  moveit_scene(ros::NodeHandle n);
  ~moveit_scene();
  void pointCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
};

moveit_scene::moveit_scene(ros::NodeHandle n)
{
  sub = n.subscribe<sensor_msgs::PointCloud2>("/camera/depth/color/points", 1, boost::bind(&moveit_scene::pointCallback, this, _1));
  pub = n.advertise<sensor_msgs::PointCloud2>("filter_clouds", 1);
}

moveit_scene::~moveit_scene()
{
}

void moveit_scene::pointCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  // Get data from msg
  // ROS_INFO("I heard");
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);

  // Filter
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);

  // Publish processed clouds
  sensor_msgs::PointCloud2::Ptr output_msg (new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*cloud_filtered, *output_msg);
  pub.publish(*output_msg);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "moveit_scene");
  ros::NodeHandle n;
  moveit_scene a(n);

  ros::spin();

  return 0;
}


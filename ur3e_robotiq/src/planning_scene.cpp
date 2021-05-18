# include <ros/ros.h>
# include <moveit/robot_model_loader/robot_model_loader.h>
# include <moveit/planning_scene/planning_scene.h>
# include <moveit_msgs/PlanningScene.h>
# include <octomap_msgs/Octomap.h>
# include <octomap_msgs/OctomapWithPose.h>
# include <geometry_msgs/Pose.h>

class Octomap2Scene{
public:
    Octomap2Scene(ros::NodeHandle n){
        sub = n.subscribe<octomap_msgs::Octomap>("/octomap_full", 1, boost::bind(&Octomap2Scene::octomapCallback, this, _1));
        pub = n.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);
    }
private:
    void octomapCallback(const octomap_msgs::Octomap::ConstPtr& octomap_msg)
    {  
        //ROS_INFO("I heard");
        geometry_msgs::Pose pose;
        pose.orientation.w = 1;
        pose.orientation.x = 0;
        pose.orientation.y = 0;
        pose.orientation.z = 0;
        pose.position.x = 0;
        pose.position.y = 0;
        pose.position.z = 0;

        moveit_msgs::PlanningScene planning_scene_msg;
        planning_scene_msg.is_diff = true;
        planning_scene_msg.world.octomap.header.frame_id = octomap_msg->header.frame_id;
        planning_scene_msg.world.octomap.header.stamp = ros::Time::now();
        planning_scene_msg.world.octomap.octomap = *octomap_msg;
        planning_scene_msg.world.octomap.origin = pose;

        pub.publish(planning_scene_msg);
    }

    ros::Subscriber sub;
    ros::Publisher pub;
};
    
int main(int argc, char **argv){
    ros::init(argc, argv, "octomap_to_planningScene");
    ros::NodeHandle n;

    Octomap2Scene a(n);

    ros::spin();
    ros::shutdown();
    return 0;
}

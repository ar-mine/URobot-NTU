# URobot-NTU

## How to install 

```
# Source global ros(optional)
$ source /opt/ros/<your_ros_version>/setup.bash

# Create a catkin workspace
$ mkdir -p catkin_ws/src && cd catkin_ws/src

# Clone this repo 
$ git clone https://github.com/ar-mine/URobot-NTU.git

# Install Gazebo Plugin for mimic
$ git clone https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins.git

# Install by using wstool
$ wstool init .
$ wstool merge -t . URobot-NTU/URobot.rosinstall
$ wstool update -t .

# Install dependencies
$ sudo apt update -qq
$ sudo apt dist-upgrade
$ rosdep update
$ cd ..
$ rosdep install --from-paths src --ignore-src -y

# Build the workspace
$ catkin build

# Activate the workspace (ie: source it)
$ source devel/setup.bash

--------------------------------------
## How to simulate with gazebo
### 1. Just control with gui 
$ roslaunch ur3e_robotiq_gazebo ur3e_gripper_bringup.launch
(Another command line)
$ rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller
### 2. Control with Moveit!
$ roslaunch ur3e_robotiq_gazebo ur3e_gripper_bringup.launch
$ roslaunch ur3e_robotiq_moveit move_group.launch sim:=true
$ roslaunch ur3e_robotiq_gazebo rviz.launch
```

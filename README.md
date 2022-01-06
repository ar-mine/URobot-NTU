# URobot-NTU

## User manual

```
# source global ros(optional)
$ source /opt/ros/<your_ros_version>/setup.bash

# create a catkin workspace
$ mkdir -p catkin_ws/src && cd catkin_ws/src

# Clone repo 
$ git clone git@github.com:ar-mine/ur_dlidar.git

# Install by using wstool
wstool init .
wstool merge -t . URobot-NTU/URobot.rosinstall
wstool update -t .

# install dependencies
$ sudo apt update -qq
$ sudo apt dist-upgrade
$ rosdep update
$ cd ..
$ rosdep install --from-paths src --ignore-src -y

# build the workspace
$ catkin build

# activate the workspace (ie: source it)
$ source devel/setup.bash

--------------------------------------
# test simulation
$ roslaunch ur3e_robotiq_gazebo ur3e_gripper_bringup.launch
(Another command line)
$ rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller

```

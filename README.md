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

# If python-pymodbus install not successful
$ sudo apt install python3-pymodbus

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
```

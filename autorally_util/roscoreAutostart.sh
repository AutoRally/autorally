#! /bin/bash
source /opt/ros/indigo/setup.bash
source /PATH_TO_CATKIN_WS/install/setup.sh

export ROS_PACKAGE_PATH=/PATH_TO_REPOSITORY:$ROS_PACKAGE_PATH
export PATH=$PATH:$ROS_ROOT/bin
roscore

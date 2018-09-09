#!/bin/sh

# source this when you launch everything on a local machine
export ROS_MASTER_URI=http://localhost:11311
export MASTER_HOSTNAME=localhost
export HOSTNAME=localhost
export ROSLAUNCH_SSH_UNKNOWN=0
# Find directory of script file to avoid hard-coded paths
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source $DIR/setupEnvVariables.sh
export CATKIN_ROOT=$DIR/../../../

#!/bin/sh

# source this when you launch everything on a local machine
export ROS_MASTER_URI=http://kezia.cc.gt.atl.ga.us:11311
export MASTER_HOSTNAME=kezia.cc.gt.atl.ga.us
export HOSTNAME=kezia.cc.gt.atl.ga.us
export ROSLAUNCH_SSH_UNKNOWN=0
# Find directory of script file to avoid hard-coded paths
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source $DIR/setupEnvVariables.sh

#!/bin/sh

# source this before you perform a distributed launch with a remote AutoRally chassis
export ROS_MASTER_URI=http://kezia.cc.gt.atl.ga.us:11311
export MASTER_HOSTNAME=kezia.cc.gt.atl.ga.us
export MASTER_USER=jake
export HOSTNAME=$(hostname)
export ROSLAUNCH_SSH_UNKNOWN=1
# Find directory of script file to avoid hard-coded paths
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source $DIR/setupEnvVariables.sh

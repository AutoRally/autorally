#!/bin/sh

# source this before you perform a distributed launch with a remote AutoRally chassis
export ROS_MASTER_URI=http://COMPUTE_BOX_HOSTNAME:11311
export MASTER_HOSTNAME=COMPUTE_BOX_HOSTNAME
export MASTER_USER=COMPUTE_BOX_USERNAME
export HOSTNAME=$(hostname)
export ROSLAUNCH_SSH_UNKNOWN=1
# Find directory of script file to avoid hard-coded paths
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source setupEnvVariables.sh

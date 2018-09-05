#!/usr/bin/env zsh

# Setup some variables
export ROS_MASTER_URI=http://COMPUTE_BOX_HOSTNAME:11311
export MASTER_HOSTNAME=COMPUTE_BOX_HOSTNAME
export MASTER_USER=COMPUTE_BOX_USERNAME
export HOSTNAME=$(hostname)
export ROSLAUNCH_SSH_UNKNOWN=1

# Source the setupEnvVariables.sh from the same directory as this file
DIR=$(builtin cd -q "`dirname "$0"`" > /dev/null && pwd)
emulate -R sh -c 'source "$DIR/setupEnvVariables.sh"'

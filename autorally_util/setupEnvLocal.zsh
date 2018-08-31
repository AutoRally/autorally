#!/usr/bin/env zsh

# Setup some variables
export ROS_MASTER_URI=http://localhost:11311
export MASTER_HOSTNAME=localhost
export HOSTNAME=localhost
export ROSLAUNCH_SSH_UNKNOWN=0

# Source the setupEnvVariables.sh from the same directory as this file
DIR=$(builtin cd -q "`dirname "$0"`" > /dev/null && pwd)
emulate -R sh -c 'source "$DIR/setupEnvVariables.sh"'

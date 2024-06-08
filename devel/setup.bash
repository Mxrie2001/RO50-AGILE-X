#!/usr/bin/env bash
# generated from catkin/cmake/templates/setup.bash.in

CATKIN_SHELL=bash

# source setup.sh from same directory as this file
_CATKIN_SETUP_DIR=$(builtin cd "`dirname "${BASH_SOURCE[0]}"`" > /dev/null && pwd)
. "$_CATKIN_SETUP_DIR/setup.sh"

export ROS_MASTER_URI=http://192.168.8.187:11311/
export ROS_HOSTNAME=192.168.8.153
export ROS_IP=192.168.8.153

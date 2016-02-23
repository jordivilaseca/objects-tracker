#!/usr/bin/env bash

# .bashrc elements
source /opt/ros/indigo/setup.bash
source ~/inhands_ws/devel/setup.bash
source ~/tracker_ws/devel/setup.bash
#export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/tracker_ws/src
export ROS_WORKSPACE='/home/ros/inhands_ws/'
export ROS_IP=192.168.2.245


exec "$@"

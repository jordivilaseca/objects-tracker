#!/bin/bash

#### Set general ros configuration ####
source /opt/ros/indigo/setup.bash
export ROS_IP=192.168.2.10

#### Set specific ros configuration ####
source ~/catkin_ws/devel/setup.bash

#### Define some util alias #####
alias inhands="ssh ros@192.168.2.245"

# Use roscore from the server
alias confmaster="ROS_MASTER_URI=http://192.168.2.245:11311/"

# See server gpu information 
alias inhandsinfo="watch nvidia-smi"

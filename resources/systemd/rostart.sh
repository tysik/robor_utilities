#!/usr/bin/env bash

export ROS_MASTER_URI=http://yb_pc:11311
export ROS_HOSTNAME=yb_pc
export ROS_HOME=/home/youbot/.ros

. /opt/ros/kinetic/setup.sh
. /home/youbot/robor_ws/devel/setup.sh

roslaunch /home/youbot/robor_ws/src/robor_utilities/launch/robor_server.launch

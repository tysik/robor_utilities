#!/bin/bash
export ROS_MASTER_URI=http://yb_pc:11311
export ROS_IP=yb_pc
export ROS_HOSTNAME=yb_pc

. /opt/ros/kinetic/setup.sh
. $HOME/robor_ws/devel/setup.sh

exec "$@"

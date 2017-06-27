#!/bin/bash
export ROS_MASTER_URI=http://yb_pc:11311
export ROS_IP=yb_${USER}
export ROS_HOSTNAME=yb_${USER}

. /opt/ros/kinetic/setup.sh
. $HOME/robor_ws/devel/setup.sh

exec "$@"

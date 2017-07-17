#!/bin/bash

echo "Starting Robor";
export ROS_MASTER_URI=http://yb_pc:11311
export ROS_HOSTNAME=yb_${USER}

roslaunch robor_utilities robor_client.launch $1:=true

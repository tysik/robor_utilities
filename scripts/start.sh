#!/bin/bash

if [ $1 == "simulator" ]; then
  echo "Starting Simulator";
else
  echo "Starting Robor";

  export ROS_MASTER_URI=http://yb_pc:11311
  export ROS_HOSTNAME=yb_${USER}
fi

roslaunch robor_utilities robor.launch $1:=true

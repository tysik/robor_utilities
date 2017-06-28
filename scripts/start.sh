#!/bin/bash

if [ $1 == "simulator" ]; then
  echo "Starting Simulator";
else
  echo "Starting Robor";

  export ROS_MASTER_URI=http://yb_pc:11311
  export ROS_IP=yb_${USER}
  export ROS_HOSTNAME=yb_${USER}

  ssh youbot@yb_pc 'pkill ros' &
  ssh youbot@yb_pc 'source $HOME/robor_ws/src/robor_utilities/scripts/youbot_env.sh && roscore' &
  sleep 5s;
fi

roslaunch robor_utilities robor.launch $1:=true

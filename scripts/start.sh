#!/bin/bash
. $HOME/robor_ws/src/robor_utilities/scripts/user_env.sh
ssh youbot@yb_pc 'pkill ros' &
ssh youbot@yb_pc 'source $HOME/robor_ws/src/robor_utilities/scripts/youbot_env.sh && roscore' &
sleep 5s
roslaunch robor_utilities robor.launch
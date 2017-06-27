#!/bin/bash

#
# DEPRECATED
#

echo "Copying libraries";
echo "-----------------";
scp $HOME/Robor/robor_ws/devel/lib/lib* youbot@yb_pc:/home/youbot/Robor/robor_ws/devel/lib

echo "Copying executables";
echo "-------------------";
scp -r $HOME/Robor/robor_ws/devel/lib/robor_utilities/ youbot@yb_pc:/home/youbot/Robor/robor_ws/devel/robor_utilities/
scp -r $HOME/Robor/robor_ws/devel/lib/robor_controllers/ youbot@yb_pc:/home/youbot/Robor/robor_ws/devel/robor_controllers/
scp -r $HOME/Robor/robor_ws/devel/lib/obstacle_detector/ youbot@yb_pc:/home/youbot/Robor/robor_ws/devel/obstacle_detector/

#
# DEPRECATED
#
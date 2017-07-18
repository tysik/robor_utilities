### Instalation

In your home folder create a catkin workspace named `robor_ws`. In the src folder add the following packages:

1. youbot_driver (https://github.com/youbot/youbot_driver/tree/hydro-devel)
2. youbot_base_driver (robor)
3. mocap_optitrack (https://github.com/ros-drivers/mocap_optitrack)
4. obstacle_detector (robor)
5. robor_controllers (robor)
6. robor_utilities (this, robor)

### Configuration

In your /etc/hosts add:
192.168.1.220   yb_pc
`<your_ip_in_roksis_network> yb_<your_username>`

In youbot /etc/hosts add:
`<your_ip_in_roksis_network> yb_<your_username>`

Configure ssh keys between both machines. When you ssh into youbot for the first time, use "ssh -oHostKeyAlgorithms='ssh-rsa' youbot@yb_pc". Otherwise you will fall into strange issues. Use:  
`ssh-copy-id youbot@yb_pc`  
in one direction, the  
`ssh-copy-id <your_username>@yb_<your_username>`  
for the other.

### Aliases

To make life easier, add following aliases to .bashrc file:  

alias rbr_start='rbr_odometry'
alias rbr_simulator='roslaunch robor_utilities robor_simulator.launch'
alias rbr_odometry='source $HOME/robor_ws/src/robor_utilities/scripts/start.sh use_odometry'
alias rbr_optitrack='source $HOME/robor_ws/src/robor_utilities/scripts/start.sh use_optitrack'
alias rbr_amcl='source $HOME/robor_ws/src/robor_utilities/scripts/start.sh use_map'
alias rbr_mapping='source $HOME/robor_ws/src/robor_utilities/scripts/start.sh do_mapping'

alias rbr_connect='ssh youbot@yb_pc'
alias rbr_env='source $HOME/robor_ws/src/robor_utilities/scripts/user_env.sh'
alias rbr_shutdown='ssh -t youbot@yb_pc "sudo systemctl poweroff"'
alias rbr_reboot='ssh -t youbot@yb_pc "sudo systemctl reboot"'

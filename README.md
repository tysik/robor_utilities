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

alias rbr_connect='ssh youbot@yb_pc'
alias rbr_simulator='roslaunch robor_utilities robor.launch simulator:=true'
alias rbr_start='source $HOME/robor_ws/src/robor_utilities/scripts/start.sh'
alias rbr_shutdown='ssh -t youbot@yb_pc "sudo shutdown -h now"'

### Date and time synchronization between two machines

1. Remove ntp and/or ntpdate on both computers
2. Install chrony on both computers
3. On computer which will be used as time server edit (or create) file /etc/chrony/chrony.conf and use the configuration provided in resources/chrony_server.conf.
4. On computer which will be synchronizing with the server edit (or create) file /etc/chrony/chrony.conf and use the configuration provided in resources/chrony_client.conf.
5. If the chrony daemon (chronyd) does not start at boot, add the following lines to the /etc/rc.local file on that machine (make sure the paths are correct):

`if [ -f /usr/sbin/chronyd -a -f /etc/chrony/chrony.conf ]; then`
`  /usr/sbin/chronyd -r -s`
`  echo "Start chronyd"`
`fi`

To check the status of chrony use `chronyc tracking`. To check the sources of chrony use `chronyc sources`.
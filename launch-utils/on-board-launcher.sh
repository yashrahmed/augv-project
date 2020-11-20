#!/bin/bash
IP_ADD=$(python3 ./launch-utils/get-ip.py)
echo "ROBOT IP ${IP_ADD}"
export ROS_MASTER_URI=http://${IP_ADD}:11311
export ROS_HOSTNAME="${IP_ADD}"
export ROS_IP="${IP_ADD}"

source ./devel/setup.sh
roslaunch augv_main on-robot.launch
#!/bin/bash

IP_ADD=$(python3 ./launch-utils/get-ip.py)
echo "SYSTEM IP = ${IP_ADD}"
echo "EXPORT variable ROS_MASTER_URI to complete setup."

export ROS_MASTER_URI=http://10.0.0.115:11311 # @toDo - This is temporary
export ROS_HOSTNAME="${IP_ADD}"
export ROS_IP="${IP_ADD}"
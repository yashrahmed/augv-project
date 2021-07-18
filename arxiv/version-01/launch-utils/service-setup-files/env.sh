#!/bin/sh

# 
# Taken from https://blog.roverrobotics.com/how-to-run-ros-on-startup-bootup/
# Copy this file to /etc/ros/env.sh

export ROS_HOSTNAME=$(hostname);
export ROS_MASTER_URI=http://$ROS_HOSTNAME:11311

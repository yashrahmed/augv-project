#!/bin/bash

sudo cp ./service-setup-files/env.sh /etc/ros/env.sh
sudo cp ./service-setup-files/launch-nodes /usr/sbin/launch-nodes
sudo cp ./service-setup-files/roscore.service /etc/systemd/system/roscore.service
sudo cp ./service-setup-files/roslaunch.service /etc/systemd/system/roslaunch.service

sudo systemctl enable roscore.service
sudo systemctl enable roslaunch.service
sudo chmod +x /usr/sbin/launch-nodes

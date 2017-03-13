#!/bin/bash


sudo chmod 777 /dev/ttyUSB0
sudo chmod 777 /dev/ttyUSB1

MASTER_IP=192.168.1.100:11311

source devel/setup.bash
export ROS_MASTER_URI=http://$MASTER_IP
roslaunch a_robot_platform config.launch 

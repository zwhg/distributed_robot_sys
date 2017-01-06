#!/bin/bash


sudo chmod 777 ttyUSB0
sudo chmod 777 ttyUSB1

IP=192.168.1.100:11311

source temp/devel/setup.bash
export ROS_MASTER_URI=http://&IP
roslaunch a_robot_platform config.launch 

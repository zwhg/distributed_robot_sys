#!/bin/bash  

MASTER_IP=192.168.1.100:11311

source googleMapping/devel_isolated/setup.bash
export ROS_MASTER_URI=http://$MASTER_IP
roslaunch cartographer_ros orange.launch

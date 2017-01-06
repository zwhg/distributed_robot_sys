#!/bin/bash  

IP=192.168.1.100:11311

source googleMapping/devel_isolated/setup.bash
export ROS_MASTER_URI=http://&IP
roslaunch cartographer_ros orange.launch

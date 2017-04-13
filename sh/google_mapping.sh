#!/bin/bash  

source ~/distributed_robot_sys/sh/ip.sh
echo $MASTER_IP

source googleMapping/devel_isolated/setup.bash
export ROS_MASTER_URI=http://$MASTER_IP
roslaunch cartographer_ros orange.launch

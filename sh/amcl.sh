#!/bin/bash

source ~/distributed_robot_sys/sh/ip.sh
echo $MASTER_IP

source location/devel/setup.bash
export ROS_MASTER_URI=http://$MASTER_IP
roslaunch amcl orange_diff.launch 

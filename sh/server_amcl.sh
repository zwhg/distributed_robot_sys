#!/bin/bash

MASTER_IP=192.168.1.100:11311

source location/devel/setup.bash
export ROS_MASTER_URI=http://$MASTER_IP
roslaunch amcl orange_diff.launch 

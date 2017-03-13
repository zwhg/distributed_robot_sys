#!/bin/bash

sudo chmod 777 /dev/ttyUSB0
sudo chmod 777 /dev/ttyUSB1

source devel/setup.bash
roslaunch a_robot_platform server_location.launch 



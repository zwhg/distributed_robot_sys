#!/bin/bash  

name=$1

source googleMapping/install_isolated/setup.bash
rosservice call /finish_trajectory "stem: '${name}'"

#!/bin/bash

# Program:
# Create ROS pkg
#	duckietown_msgs, geometry_msgs, roscpp, rospy, 
#	std_msgs, sensor_msgs, cv_bridge
# History:
# 2017/03/09	Monica	v1.2

### Set up pkg name after calling this file.

if [ $# -gt 0 ]; then
	catkin_create_pkg $1 duckietown_msgs geometry_msgs roscpp rospy std_msgs sensor_msgs cv_bridge
	cp ~/duckietown/catkin_ws/src/arg_nctu/monica/README.md $1/README.md
	mkdir $1/launch
else
	echo 'Please Set File and PKG Name.'
fi
#!/usr/bin/env bash
source ~/duckietown/catkin_ws/devel/setup.bash
export ROS_HOSTNAME=$HOSTNAME.local
exec "$@"

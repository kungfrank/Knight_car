#!/usr/bin/env bash
echo "Building machines file"
make
echo "Activating ROS"
source /opt/ros/indigo/setup.bash
echo "Setting up PYTHONPATH"
export PYTHONPATH=/home/ubuntu/duckietown/catkin_ws/src:$PYTHONPATH
echo "Activating development"
source ~/duckietown/catkin_ws/devel/setup.bash
echo "Setup ROS_HOSTNAME"
export ROS_HOSTNAME=$HOSTNAME.local
export DUCKIETOWN_ROOT=$HOME/duckietown
exec "$@" #Passes arguments. Need this for ROS remote launching to work.

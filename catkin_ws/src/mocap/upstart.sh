#!/bin/bash

echo "Open LCM Tunnel"
bot-lcm-tunnel &
echo "...done."

echo "PiCamera ROS to LCM"
roslaunch mocap camera.launch &&
echo "...done."

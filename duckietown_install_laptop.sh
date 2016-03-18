#!/usr/bin/env bash
set -e


#Install open-ssh and avahi
sudo apt-get install openssh-server avahi-daemon avahi-discover avahi-utils -y

#Install some GUI tools for git
sudo apt-get install gitg kdiff3 -y

#Install byobu
sudo apt-get install byobu -y

#Install sublime text editor
sudo add-apt-repository ppa:webupd8team/sublime-text-3
sudo apt-get update
sudo apt-get install sublime-text-installer -y

# packages for the IMU
sudo apt-get install ros-indigo-phidgets-drivers ros-indigo-imu-tools -y

# Additional ROS pkgs
sudo apt-get install ros-indigo-{tf-conversions,cv-bridge,image-transport,camera-info-manager,theora-image-transport} -y

# List of additional system pkgs
# sudo apt-get install libyaml-cpp-dev


#!/usr/bin/env bash
#Update and upgrade Ubuntu
sudo apt-get update
sudo apt-get upgrade

#Install ROS
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116
sudo apt-get update
sudo apt-get install ros-indigo-desktop-full

sudo rosdep init
rosdep update

echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt-get install python-rosinstall

# Install latest version of git
sudo apt-add-repository ppa:git-core/ppa
sudo apt-get update
sudo apt-get install git

# Checkout duckietown/Software repo from github
# mkdir -p ~/duckietown
# cd ~/duckietown
# git clone https://github.com/duckietown/Software.git .
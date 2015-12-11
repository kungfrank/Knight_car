#! /bin/sh
#

# Run this script on fresh ubuntu installation (as super user, of
# course)

# TODO: Resize SD partition


# add ROS repository
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -

# add ROS repository
apt-get update
apt-get upgrade

# install essentials
apt-get -y install \
    linux-firmware \
    build-essential \
    unzip \
    wireless-tools \
    emacs \
    libraspberrypi-dev \
    libraspberrypi-bin


# install ROS
apt-get install ros-indigo-ros-base

# Install third party packages

# NOIP2
# cd /usr/local/src/
# wget http://www.no-ip.com/client/linux/noip-duc-linux.tar.gz
# tar xf noip-duc-linux.tar.gz
# cd noip-2.1.9-1/
# make install
# cd ../
# rm noip-duc-linux.tar.gz


# pigpio
wget abyz.co.uk/rpi/pigpio/pigpio.zip
unzip pigpio.zip
cd PIGPIO
make install
cd ../
rm pigpio.zip


# Couldn't find the file
# motor hat
wget https://github.com/adafruit/Adafruit-Motor-HAT-Python-Library/archive/master.zip
unzip master.zip
cd Adafruit-Motor-HAT-Python-Library-master
python setup.py install
cd ../
rm master.zip

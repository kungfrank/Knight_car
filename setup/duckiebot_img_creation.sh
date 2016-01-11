#!/usr/bin/env bash
#All the commands used to get from 2015-04-06-ubuntu-trusty.img to 2016-01-10_duckiebot.img

# Update and upgrade # This might take a while
sudo apt-get update -y
sudo apt-get upgrade -y

# Install openssh-server for ssh
sudo apt-get install openssh-server -y

# Install avahi for hostname.local access
sudo apt-get install avahi-daemon avahi-discover avahi-utils -y

# Sync the time
sudo ntpdate -u us.pool.ntp.org

# === Extra stuff ===
sudo apt-get install -y build-essential git python python-dev ipython python-pip
sudo apt-get install -y vim htop byobu libav-tools curl

# === Install ROS following http://wiki.ros.org/indigo/Installation/UbuntuARM ===
# Setup locale, deb, and keys
sudo update-locale LANG=C LANGUAGE=C LC_ALL=C LC_MESSAGES=POSIX
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
# Install ROS
sudo apt-get update
sudo apt-get install ros-indigo-ros-base -y
# Update rosdep
sudo apt-get install python-rosdep
sudo rosdep init
rosdep update

# === Bus and i2c ===
sudo apt-get install -y i2c-tools python-smbus
sudo usermod -a -G i2c ubuntu
# Trigger the rule (reboot will do this too)
sudo udevadm trigger 

# === PiCamera Stuff === #
sudo apt-get install --reinstall libraspberrypi0 libraspberrypi-{bin,dev,doc}
# Append lines to /boot/config.txt to enable the PiCamera
printf '#Enable PiCamera Interface\n %s\n %s\n' 'start_x=1' 'gpu_mem=256' | sudo tee -a /boot/config.txt
# Create custom rule to enable camera
printf 'SUBSYSTEM=="vchiq",GROUP="video",MODE="0660"' | sudo tee /etc/udev/rules.d/10-vchiq-permissions.rules
# Add user to the video group
sudo usermod -a -G video ubuntu

# Install python picamera drivers
sudo pip install picamera
sudo pip install "picamera[array]"

# ==== ros_on_ras_with_cam_2016-01-09_1846.img at this point ==== #
sudo apt-get install gfortran -y

# ==== Wireless settings ==== #
# Install firmware and tools
sudo apt-get install wireless-tools wpasupplicant linux-firmware
# Setup /etc/network/interfaces
printf 'allow-hotplug wlan0\n auto wlan0\n iface wlan0 inet manual\n wpa-roam /etc/wpa_supplicant/wpa_supplicant.conf\n iface default inet dhcp' | sudo tee -a /etc/network/interfaces
# Setup /etc/wpa_supplicat.conf header
printf 'ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev\n update_config=1' | sudo tee -a /etc/wpa_supplicant/wpa_supplicant.conf
# Setup duckietown network SSID
printf 'network={\n ssid="duckietown"\n scan_ssid=1\n psk="quackquack"\n priority=10\n}' | sudo tee -a /etc/wpa_supplicant/wpa_supplicant.conf

# Setup authorized keys for ssh access for duckietown
cd ~
mkdir .ssh
chmod g-rwx,o-rwx .ssh 
wget -O .ssh/authorized_keys https://www.dropbox.com/s/pxyou3qy1p8m4d0/duckietown_key1.pub?dl=1

#Add passwordless sudo for user ubuntu:
# $ sudo visudo
# Add line at the end:
# ubuntu ALL=(ALL) NOPASSWD: ALL 

# Select option (1) for byobu.

# Install additional ROS pkgs to apt-get
sudo apt-get install ros-indigo-{tf-conversions,cv-bridge,image-transport,camera-info-manager,theora-image-transport}

# List of additional system pkgs
sudo apt-get install libyaml-cpp-dev

# Remove the net rules to forget the mac address of ethernet and wireless cards
rm /etc/udev/rules.d/70-persistent-net.rules

# === Duckietown Img ==== #

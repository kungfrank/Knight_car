#! /bin/sh
#
# Ubuntu installation script for RPi 2.

# The best starting point is to follow this guide:
# https://wiki.ubuntu.com/ARM/RaspberryPi

# Short instructions to put Ubuntu image on SD card:
# 1. Download Ubuntu image from 
#    http://www.finnie.org/software/raspberrypi/2015-04-06-ubuntu-trusty.zip

if [ ! -f 2015-04-06-ubuntu-trusty.img ];
then
    echo "Downloading Ubuntu image"
    wget http://www.finnie.org/software/raspberrypi/2015-04-06-ubuntu-trusty.zip

    echo "Unpacking Ubuntu image"
    unzip 2015-04-06-ubuntu-trusty.zip
fi

# 2. burn image onto sd card
#    (Substitute X with your sd card device;
#    Ignore digits at the end of device name)

echo "About to burn 2015-04-06-ubuntu-trusty.img onto $1"
read -p "Please confirm if you agree [y/N]" yn
case $yn in
     [Yy]* ) echo "Burning Ubuntu image onto SD card"; dd if=2015-04-06-ubuntu-trusty.img of=$1; break;;
     * ) echo "I didn't get a positive answer: exiting"; exit;;
esac

echo "Done"

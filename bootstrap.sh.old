#!/bin/bash
set -e
set -x

echo "First, let's set the date correctly"
sudo ntpdate -u us.pool.ntp.org 


echo "I expect DUCKIE_ROOT to be set"
echo DUCKIE_ROOT=$DUCKIE_ROOT

echo "I will create a file config.sh here"
config=$DUCKIE_ROOT/config.sh
touch config

echo I will include that file in .bashrc

echo "source $config" >> ~/.bashrc



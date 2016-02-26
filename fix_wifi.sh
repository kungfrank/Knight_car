#!/usr/bin/env bash
# installs fix from https://launchpad.net/ubuntu/+source/ifupdown/0.7.47.2ubuntu4.4
# setting up proposed repo comes https://wiki.ubuntu.com/Testing/EnableProposed

set -e

if grep -q "trusty-proposed" /etc/apt/sources.list
then
    echo "you already got trusty-proposed repos enabled!"
else
    echo "adding trusty-proposed!"
    echo 'deb http://ports.ubuntu.com trusty-proposed restricted main multiverse universe' >> /etc/apt/sources.list
fi

if [ -f /etc/apt/preferences.d/proposed-updates ];
then
    echo "you already have a preference file in place!"
else
    echo 'Package: *\nPin: release a=trusty-proposed\nPin-Priority: 400\n' >>  /etc/apt/preferences.d/proposed-updates
fi

#dpkg -s ifupdown | grep Version
sudo apt-get update
sudo apt-get install ifupdown/trusty-proposed

echo 'please reboot your duckie!'

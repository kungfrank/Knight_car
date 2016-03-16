#!/bin/bash
#set -e
set -x
sudo route del default gw 192.168.13.1
sudo route add default gw 10.0.1.1 
 


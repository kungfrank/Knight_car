#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from dynamic_pid.cfg import PIDConfig

import time

def callback(config, level):
   rospy.loginfo("Reconfigure Request: {P_param}, {I_param},{D_param}".format(**config))

   global p 
   global i 
   global d 
   p = config.P_param
   i = config.I_param
   d = config.D_param

   return config

if __name__ == "__main__":
  rospy.init_node("dynamic_pid", anonymous = True)

  global p 
  global i 
  global d 
  srv = Server(PIDConfig, callback)
  while(1):
    print "P = ",p,"\tI = ",i,"\tD = ",d
    time.sleep(1)
  rospy.spin()

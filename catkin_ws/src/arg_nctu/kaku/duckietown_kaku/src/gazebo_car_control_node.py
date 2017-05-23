#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Point
import time
import math
from Adafruit_MotorHAT import Adafruit_MotorHAT

class gazebo_car_control_node(object):
    def __init__(self):
	    self.node_name = "gazebo_car_control_node"
	    self.active = True
		self.i = 0
		self.motorhat = Adafruit_MotorHAT(addr=0x60)
		self.gazebo_car_control_L = self.motorhat.getMotor(1)
		self.gazebo_car_control_R = self.motorhat.getMotor(2)

		self.sub_control_value = rospy.Subscriber("~control_value", Point, self.cbControl_value, queue_size=1)
		

    def cbControl_value(self, msg):
		
		u_R = msg.x
		u_L = msg.y

		if u_R > 0:
			self.gazebo_car_control_R.run(Adafruit_MotorHAT.FORWARD)
			self.gazebo_car_control_R.setSpeed(u_R)
		else:
			self.gazebo_car_control_R.run(Adafruit_MotorHAT.BACKWARD)
			self.gazebo_car_control_R.setSpeed(u_R)

		if u_L > 0:
			self.gazebo_car_control_R.run(Adafruit_MotorHAT.FORWARD)
			self.gazebo_car_control_R.setSpeed(u_L)
		else:
			self.gazebo_car_control_R.run(Adafruit_MotorHAT.BACKWARD)
			self.gazebo_car_control_R.setSpeed(u_L)

		
		# self.gazebo_car_control_.run(Adafruit_MotorHAT.RELEASE)

 
    def onShutdown(self):
        rospy.loginfo("[gazebo_car_control_node] Shutdown.")

if __name__ == '__main__': 
    rospy.init_node('gazebo_car_control_node',anonymous=False)
    gazebo_car_control_node = gazebo_car_control_node()
    rospy.on_shutdown(gazebo_car_control_node.onShutdown)
    rospy.spin()


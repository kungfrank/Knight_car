#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Point
from std_msgs.msg import Int64
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
		self.threshold = 0
		self.sub_control_value = rospy.Subscriber("/gazebo_sub_jointstate/control_value", Point, self.cbControl_value, queue_size=1)
		self.sub_threshold_value = rospy.Subscriber("/gazebo_sub_jointstate/threshold_value", Int64, self.cbThreshold_value, queue_size=1)
		
    def cbThreshold_value(self, msg):
		self.threshold = msg.data

    def cbControl_value(self, msg):
		
		u_R = int(msg.x)
		u_L = int(msg.y)
		print "u_R = ",u_R,"\tu_L = ",u_L
		# if u_R > 200:
		# 	u_R = 200
		# if u_L > 200:
		# 	u_L = 200
		# if u_R < -200:
		# 	u_R = -200
		# if u_L < -200:
		# 	u_L = -200


		if u_R > self.threshold:
			self.gazebo_car_control_R.setSpeed(u_R)
			self.gazebo_car_control_R.run(Adafruit_MotorHAT.FORWARD)
		elif u_R < -(self.threshold):
			self.gazebo_car_control_R.setSpeed(-u_R)
			self.gazebo_car_control_R.run(Adafruit_MotorHAT.BACKWARD)
		else:
			self.gazebo_car_control_R.setSpeed(0)
			self.gazebo_car_control_R.run(Adafruit_MotorHAT.BACKWARD)

		if u_L > self.threshold:
			self.gazebo_car_control_L.setSpeed(u_L)
			self.gazebo_car_control_L.run(Adafruit_MotorHAT.FORWARD)
		elif u_L < -(self.threshold):
			self.gazebo_car_control_L.setSpeed(-u_L)
			self.gazebo_car_control_L.run(Adafruit_MotorHAT.BACKWARD)
		else:
			self.gazebo_car_control_L.setSpeed(0)
			self.gazebo_car_control_L.run(Adafruit_MotorHAT.FORWARD)

		
		# self.gazebo_car_control_.run(Adafruit_MotorHAT.RELEASE)

 
    def onShutdown(self):
		self.gazebo_car_control_R.setSpeed(0)
		self.gazebo_car_control_R.run(Adafruit_MotorHAT.BACKWARD)
		self.gazebo_car_control_L.setSpeed(0)
		self.gazebo_car_control_L.run(Adafruit_MotorHAT.FORWARD)
		rospy.loginfo("[gazebo_car_control_node] Shutdown.")

if __name__ == '__main__': 
    rospy.init_node('gazebo_car_control_node',anonymous=False)
    gazebo_car_control_node = gazebo_car_control_node()
    rospy.on_shutdown(gazebo_car_control_node.onShutdown)
    rospy.spin()


#!/usr/bin/env python
import rospy
import numpy as np
import math
from duckietown_msgs.msg import  Twist2DStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion
import sys
import time
import threading
class mocap_pose(object):
	def __init__(self):
		self.node_name = rospy.get_name()

		self.active = True
		self.out_of_boundary = False

		# Publicaiton
		self.pub_car_cmd = rospy.Publisher("~car_cmd",Twist2DStamped,queue_size=1)
		self.sub_joystick_car_cmd = rospy.Subscriber("~joystick_car_cmd",Twist2DStamped, self.cbJoystick,queue_size=1)
		self.sub_vehicle_position = rospy.Subscriber("~vehicle_position", Point, self.cbPosition, queue_size=1)
		self.sun_vehicle_orientation = rospy.Subscriber("~vehicle_orientation", Quaternion, self.cbOrientation, queue_size=1)

		# safe shutdown
		rospy.on_shutdown(self.custom_shutdown)

		# timer
		rospy.loginfo("[%s] Initialized " %(rospy.get_name()))

	def custom_shutdown(self):
		rospy.loginfo("[%s] Shutting down..." %self.node_name)

		# Send stop command
		car_control_msg = Twist2DStamped()
		car_control_msg.v = 0.0
		car_control_msg.omega = 0.0
		self.publishCmd(car_control_msg)
		rospy.sleep(0.5) #To make sure that it gets published.
		rospy.loginfo("[%s] Shutdown" %self.node_name)

	def publishCmd(self, car_cmd_msg):

		self.pub_car_cmd.publish(car_cmd_msg)
	
	def cbJoystick(self, car_cmd_msg):

		if self.out_of_boundary == False:
			car_control_msg = Twist2DStamped()
			car_control_msg.v = car_cmd_msg.v
			car_control_msg.omega = car_cmd_msg.omega
			self.publishCmd(car_control_msg)
		else:
			car_control_msg = Twist2DStamped()
			car_control_msg.v = 0
			car_control_msg.omega = 0
			self.publishCmd(car_control_msg)

	def cbPosition(self, pose_msg):

		xl = 0;
		xh = 1.5;
		yl = 0;
		yh = 1.5;

		if pose_msg.x < xl or pose_msg.x > xh or pose_msg.y < yl or pose_msg.y > yh:
			self.out_of_boundary == True
			rospy.loginfo("Duckiebot out of tiles bundaries")
		else:
			self.out_of_boundary == False
			rospy.loginfo("Joystick available")
	def cbOrientation(self , pose_o_msg) :

		quaternion = ( pose_o_msg.x, pose_o_msg.y, pose_o_msg.z, pose_o_msg.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
		roll = euler[0]
		pitch = euler[1]
		yaw = euler[2]
		
		print (roll, pitch, yaw)


if __name__ == "__main__":
	rospy.init_node("mocap_pose",anonymous=False)
	mocap_pose_node = mocap_pose()
	rospy.spin()

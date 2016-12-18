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
		self.motion_to_target = False
		self.pos = Point();
		self.target = Point();
		self.yaw = 0; 

		# Publicaiton
		self.pub_car_cmd = rospy.Publisher("~car_cmd",Twist2DStamped,queue_size=1)
		#self.sub_joystick_car_cmd = rospy.Subscriber("~joystick_car_cmd",Twist2DStamped, self.cbJoystick,queue_size=1)
		self.sub_vehicle_position = rospy.Subscriber("~vehicle_position", Point, self.cbPosition, queue_size=1)
		self.sun_vehicle_orientation = rospy.Subscriber("~vehicle_orientation", Quaternion, self.cbOrientation, queue_size=1)
		self.sub_target_position = rospy.Subscriber("~target_position", Point, self.cbTargetn, queue_size=1)

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

		self.pos.x = pose_msg.x
		self.pos.y = pose_msg.y
		self.pos.z = pose_msg.z

	def cbOrientation(self , pose_o_msg):

		quaternion = ( pose_o_msg.x, pose_o_msg.y, pose_o_msg.z, pose_o_msg.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
		roll = euler[0]
		pitch = euler[1]
		yaw = euler[2]
		self.yaw = yaw
		print (roll, pitch, yaw)

	def cbTarget(self, pose_msg):

		rospy.loginfo("targe x : [%d] " %pose_msg.x)
		self.Forward_x(pose_msg.x)

	def Spinning(self, direction):

		while(abs(self.yaw - direction) >= 5):
			car_control_msg = Twist2DStamped()
			car_control_msg.v = 1
			car_control_msg.omega = 0.5
			self.publishCmd(car_control_msg)
			rospy.sleep(0.5)
			car_control_msg.v = 0
			car_control_msg.omega = 0
			self.publishCmd(car_control_msg)

	def Forward_x(self, target_x):

		if(self.pos.x >= target_x):
			self.Spinning(0)
			while(self.pos.x > target_x):
				car_control_msg.v = 1
				car_control_msg.omega = 0
				self.publishCmd(car_control_msg)
				rospy.sleep(0.5)
				car_control_msg.v = 0
				car_control_msg.omega = 0
				self.publishCmd(car_control_msg)

if __name__ == "__main__":
	rospy.init_node("mocap_pose",anonymous=False)
	mocap_pose_node = mocap_pose()
	rospy.spin()

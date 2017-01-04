#!/usr/bin/env python
import rospy
import roscpp
import numpy as np
import math
from duckietown_msgs.msg import  Twist2DStamped
from geometry_msgs.msg import Point, Quaternion, Pose
import tf
import sys
import time
import threading
class mocap_pose(object):
	def __init__(self):
		self.node_name = rospy.get_name()
		# vehicle position update by mocap
		self.position = Point()
		# vehicle yaw
		self.yaw = 0

		# Publicaiton
		self.pub_car_cmd_ = rospy.Publisher("~car_cmd",Twist2DStamped,queue_size=1)
		# Subscription
		self.sub_vehicle_pose_ = rospy.Subscriber("~vehicle_pose", Pose, self.cbPose, queue_size=1)

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
		self.pub_car_cmd_.publish(car_cmd_msg)
		rospy.sleep(0.5) #To make sure that it gets published.
		rospy.loginfo("[%s] Shutdown" %self.node_name)
	
	def cbPose(self, pose_msg):
		# read vehicle position form mocap pose message
		self.position.x = pose_msg.position.x
		self.position.y = pose_msg.position.y
		self.position.z = pose_msg.position.z
		# convert quaternion of mocap pose message to rpy
		quaternion = (pose_msg.x, pose_msg.y, pose_msg.z, pose_msg.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
		#roll = euler[2] * 180/math.pi
		#pitch = euler[1] * 180/math.pi
		self.yaw  = 180 - euler[0] * 180/math.pi

		#initial a target point for test
		target_point = self.set_target_point(0.5, 0.5, 0)
		target_yaw = self.get_yaw_two_point(self.position, target_point)

		# vehicle turn by yaw angle difference 
		if(abs(self.yaw - target_yaw) >= 10):
			self.publish_car_cmd(0.2, (self.yaw - target_yaw) / 180 + 2), 0.5)

	def set_target_point(self, x, y, z):
		# set a target_point
		target_point = Point()
		target_point.x = x
		target_point.y = y
		target_point.z = z

		return targe_point

	def get_yaw_two_point(self, source_point, target_point):
		# calculate arctan(in rad) of two point
		dx = target_point.x - source_point.x
		dy = target_point.y - source_point.y
		yaw = math.atan(dy/dx)
		print 'original yaw: ', yaw
		# rad compensation
		if( dx > 0):
			yaw = yaw
		elif( dx == 0 and dy == 0):
			yaw = 0
		elif( dx == 0 and dy > 0):
			yaw =  math.pi / 2
		elif( dx == 0 and dy < 0):
			yaw =  -math.pi / 2
		elif( dx < 0 and dy < 0):
			yaw = yaw - math.pi;
		elif( dx < 0 and dy >= 0):
			yaw = yaw + math.pi; 
		print 'compensation yaw: ', yaw

		return yaw

	def publish_car_cmd(self, v, omega, duration):
		# publish car command
		car_cmd_msg = Twist2DStamped()
		car_cmd_msg.v = v
		car_cmd_msg.omega = omega
		self.pub_car_cmd_.publish(car_cmd_msg)
		rospy.sleep(duration)
		# stop 1s
		car_cmd_msg.v = 0
		car_cmd_msg.omega = 0
		self.pub_car_cmd_.publish(car_cmd_msg)
		rospy.sleep(0)		
    
if __name__ == "__main__":
	rospy.init_node("mocap_matrol",anonymous=False)
	mocap_matrol_node = mocap_matrol()
	rospy.spin()

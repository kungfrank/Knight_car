#!/usr/bin/env python
import rospy
import roscpp
import numpy as np
import math
from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped
import tf
import sys
import time
import tf2_ros
class state_pose_publisher(object):
	def __init__(self):
		self.node_name = rospy.get_name() 
		#print 1
		# Publicaiton
		#self.pub_state_pose_ = rospy.Publisher("~car_cmd",Twist2DStamped,queue_size=1)
		self.pub_state_pose_ = tf2_ros.TransformBroadcaster()
		# Subscription
		self.sub_pose_ = rospy.Subscriber("~pose", Pose, self.cbPose, queue_size=1)
		# safe shutdown
		rospy.on_shutdown(self.custom_shutdown)

		# timer
		rospy.loginfo("[%s] Initialized " %(rospy.get_name()))

	def custom_shutdown(self):

		rospy.loginfo("[%s] Shutting down..." %self.node_name)
		rospy.sleep(0.5) #To make sure that it gets published.
		rospy.loginfo("[%s] Shutdown" %self.node_name)

	def cbPose(self, pose_msg):
		#print 1
		trans_msg = TransformStamped()
		trans_msg.header.frame_id = 'world'
		trans_msg.child_frame_id = 'duckiebot'
		trans_msg.transform.translation.x = pose_msg.position.x
		trans_msg.transform.translation.y = pose_msg.position.y
		#trans_msg.transform.translation.z = pose_msg.position.z
		trans_msg.transform.translation.z = 0


		quaternion = ( pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
		#print euler
		roll = math.pi - euler[2] 
		pitch = math.pi - euler[1] 
		yaw = math.pi - euler[0]
		#print roll, pitch, yaw
		new_quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
		trans_msg.transform.rotation.x = new_quaternion[0]
		trans_msg.transform.rotation.y = new_quaternion[1]
		trans_msg.transform.rotation.z = new_quaternion[2]
		trans_msg.transform.rotation.w = new_quaternion[3]
		self.pub_state_pose_.sendTransform(trans_msg) 

if __name__ == "__main__":
	rospy.init_node("state_pose_publisher",anonymous=False)
	state_pose_publisher_node = state_pose_publisher()
	rospy.spin()

#!/usr/bin/python

import RPi.GPIO as GPIO
import rospy
import tf
import numpy as np

from math import sin, cos, pi
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose
from std_msgs.msg import Float64MultiArray


class Odometry(object):
	def __init__(self):
		self.node_name = rospy.get_name()
		self.br = tf.TransformBroadcaster()
		self.CPR = 70 # encoder pusle per round 2*75
		self.radius = 0.0215 # radius of wheel in meter
		self.width = 0.22 # width of robot in meter
		self.x = 0 # robot position x in meter
		self.y = 0 # robot position y in meter
		self.theta = 0 # robot pose theta in radian
		self.encoder_pos_L = 0 # post left wheel encoder
		self.encoder_pos_R = 0 # post right wheel encoder
		self.encoder_pre_L = 0 # present left wheel encoder
		self.encoder_pre_R = 0 # present right wheel encoder
        	self.R = 0
		self.P = 0
		# setup pi GPIO
		GPIO.setmode(GPIO.BCM)
		GPIO.setup(22, GPIO.IN)
		GPIO.setup(23, GPIO.IN)
		GPIO.setup(24, GPIO.IN)
		GPIO.setup(27, GPIO.IN)
		
		# interrupt callback functions
		GPIO.add_event_detect(23, GPIO.RISING, callback = self.Encoder_L)
		GPIO.add_event_detect(27, GPIO.RISING, callback = self.Encoder_R)

		# subscriber
		self.sub_RP = rospy.Subscriber("~orientationRP", Float64MultiArray, self.cbOrientation)
		rospy.on_shutdown(self.custom_shutdown) # shutdown method
		self.update_timer = rospy.Timer(rospy.Duration.from_sec(1), self.update) # timer for update robot pose, update rate = 1 Hz

		rospy.loginfo("[%s] Initialized " %self.node_name)

	def Encoder_L(self, channel):
		if GPIO.input(24):
			self.encoder_pre_L += 1
			#print "L forward! "
		else:
			self.encoder_pre_L -= 1
			#print "L backword "
	def Encoder_R(self, channel):
		if not GPIO.input(22):
			self.encoder_pre_R += 1
			#print "R forward! ",self.encoder_pre_R - self.encoder_pos_R
		else:
			self.encoder_pre_R -= 1
			#print "R backward "

	def custom_shutdown(self):
		rospy.loginfo("[%s] Shutting down..." %self.node_name)

	def update(self, event): # update robot pose method, which will execute repeatedly
		self.dphi_R = float(self.encoder_pre_R - self.encoder_pos_R) / self.CPR * (2 * pi) * self.radius  # without float, you will get zero since int division!
		self.dphi_L = float(self.encoder_pre_L - self.encoder_pos_L) / self.CPR * (2 * pi) * self.radius
		self.theta += (self.dphi_R - self.dphi_L) / self.width
		self.x += cos(self.theta - (self.dphi_R - self.dphi_L) / (2 * self.width)) * (self.dphi_R + self.dphi_L) / 2
		self.y += sin(self.theta - (self.dphi_R - self.dphi_L) / (2 * self.width)) * (self.dphi_R + self.dphi_L) / 2
		self.theta = self.theta % (2 * pi)
		#print "dphi_R:", self.dphi_R," dphi_L:", self.dphi_L # uncomment to print the distance wheels traveled
		self.encoder_pos_L = self.encoder_pre_L
		self.encoder_pos_R = self.encoder_pre_R # encoder information refrash
		# send tf from base_link to world
		self.br.sendTransform((self.x, self.y, 0), # to 3d translation
                                tf.transformations.quaternion_from_euler(self.R, self.P, self.theta), # to 3d rotation
                                rospy.Time.now(), # timestamp
                                "base_link", # robot frame
                                "map") # base frame
		print "x=", self.x," y=", self.y, " theta=" ,self.theta  # uncomment to print the position and pose of the robot
	# Mark the track the robot pass through, still has problem not solved
	'''def pubPointMarker(self,marker_id , rgba = [0.6, 0.6, 0, 1], pose = [0,0,0,0,0,0,1], frame_id = '/map'):
		marker = Marker()
		marker.header.frame_id = "/map"
		marker_type = marker.POINTS
		marker.lifetime = rospy.Duration()
		marker.scale.x = 0.01
		marker.scale.y = 0.01
		marker.scale.z = 0.01
		marker.id = marker_id

		marker.color.r, marker.color.g, marker.color.b, marker.color.a = tuple(rgba)
		p = Point()
		p.x = self.x
		p.y = self.y
		p.z = 0
 		marker.points.append(p)
        
		marker_pose = Pose()
		marker_pose.position.x = pose[0]
		marker_pose.position.y = pose[1]
		marker_pose.position.z = pose[2]
		marker_pose.orientation.x = pose[3]
		marker_pose.orientation.y = pose[4]
		marker_pose.orientation.z = pose[5]
		marker_pose.orientation.w = pose[6]
		marker.pose = marker_pose
		self.pointMarkerPub.publish(marker)'''
	def cbOrientation(self, msg):
		self.R = msg.data[0]
		self.P = msg.data[1]
if __name__ == "__main__":
	rospy.init_node("odometry", anonymous = False)
	odometry_node = Odometry()
	rospy.spin()

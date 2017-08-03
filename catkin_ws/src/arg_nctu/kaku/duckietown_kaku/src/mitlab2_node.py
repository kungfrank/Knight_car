#!/usr/bin/env python
import rospy
import roscpp
import numpy as np
import math
from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Int64
import tf
import sys
import time
import tf2_ros
class mitlab2_node(object):
	def __init__(self):
		self.node_name = rospy.get_name() 
		self.scale= rospy.get_param("~scale")
		self.shift_x = rospy.get_param("~shift_x")
		self.shift_y = rospy.get_param("~shift_y")

		self.x = 0
		self.y = 0
		self.th = 0
		self.d_PerCount = ((3.14159265 * 0.13) / 5940)*0.3/0.635
		self.Length =  0.17
		self.previousR = 0
		self.previousL = 0
		self.pretime = 0
		self.pathDistance = 0

		self.kp = 300
		self.kd = 10

		self.pub_state_pose_ = tf2_ros.TransformBroadcaster()
		self.vis_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10, latch=True)
		self.pub_car_cmd = rospy.Publisher('car_cmd', Point, queue_size=1)
		# Subscription
		self.sub_encoder = rospy.Subscriber("/encoder", Point, self.cbEncoder, queue_size=1)

		self.publish_marker()
		# safe shutdown
		rospy.on_shutdown(self.custom_shutdown)
		# timer
		rospy.loginfo("[%s] Initialized " %(rospy.get_name()))

	def custom_shutdown(self):

		rospy.loginfo("[%s] Shutting down..." %self.node_name)
		rospy.sleep(0.5) #To make sure that it gets published.
		rospy.loginfo("[%s] Shutdown" %self.node_name)

	def cbEncoder(self, encoder_msg):
		time_now = time.time()
		time_inteval = time_now - self.pretime

		delta_R = (encoder_msg.y - self.previousR) * self.d_PerCount
		delta_L= (encoder_msg.x - self.previousL) * self.d_PerCount
		delta_S = (delta_R + delta_L)/2

		R_v = delta_R / time_inteval
		L_v = delta_L / time_inteval
		# print "encoder_msg.x = ",encoder_msg.x," encoder_msg.y = ",encoder_msg.y
		#print "previousR = ",self.previousR," previousL = ",self.previousL

		self.th = ((delta_R - delta_L)/self.Length + self.th)
		delta_x = delta_S * math.cos(self.th)
		delta_y = delta_S * math.sin(self.th)
		self.x = delta_x + self.x
		self.y = delta_y + self.y

		delta_v_R = delta_R / time_inteval
		delta_v_L = delta_L / time_inteval
		self.pathDistance += math.sqrt(delta_x**2 + delta_y**2)

		# print "x = ", self.x, " y = ", self.y, " pathDistance = ",self.pathDistance
		# print "delta_v_R = ", delta_v_R, " delta_v_L = ", delta_v_L
		self.trajectU(delta_v_R, delta_v_L, time_inteval)
	
		# print "x = ",self.x," y = ",self.y," th = ",self.th	
		
		trans_msg = TransformStamped()
		trans_msg.header.frame_id = 'world'
		trans_msg.child_frame_id = 'duckiebot'
		trans_msg.transform.translation.x = self.x 
		trans_msg.transform.translation.y = self.y 
		trans_msg.transform.translation.z = 0

		new_quaternion = tf.transformations.quaternion_from_euler(0, 0, self.th)
		trans_msg.transform.rotation.x = new_quaternion[0]
		trans_msg.transform.rotation.y = new_quaternion[1]
		trans_msg.transform.rotation.z = new_quaternion[2]
		trans_msg.transform.rotation.w = new_quaternion[3]
		self.pub_state_pose_.sendTransform(trans_msg)

		point = Point()	
		point.x = trans_msg.transform.translation.x 
		point.y = trans_msg.transform.translation.y 
		point.z = trans_msg.transform.translation.z
	


		self.previousR = encoder_msg.y
		self.previousL = encoder_msg.x

		self.pretime = time_now 

	def trajectU(self, c_R, c_L, time_inteval):
		if (self.pathDistance < 1.0): 
			t_R, t_L = self.targetV(0.2, 0)
			print " straight line"
		elif (self.pathDistance < (1.0 + 0.25 *math.pi)):
			t_R, t_L = self.targetV(0.2, 3.5)
			print " Hemicircle line"
		elif (self.pathDistance < (1.0 + 0.25 *math.pi + 1)):
			t_R, t_L = self.targetV(0.2, 0)
			print " back straight line"
		else:
			t_R, t_L = self.targetV(0, 0)

		self.doPID(c_R, c_L, t_R, t_L, time_inteval)

	def doPID(self, c_R, c_L, t_R, t_L, time_inteval):
		err_R = t_R - c_R
		err_L = t_L - c_L

		# u_R = int(self.kp * err_R + self.kd * err_R/time_inteval)
		# u_L = int(self.kp * err_L + self.kd * err_L/time_inteval)
		u_R = int(self.kp * err_R )
		u_L = int(self.kp * err_L )

		control_value_msg = Point()
		control_value_msg.x = u_R
		control_value_msg.y = u_L
		control_value_msg.z = 0
		self.pub_car_cmd.publish(control_value_msg)

	def targetV(self,avgV, k):
		targetV_R = avgV + avgV * (self.Length/2) *k
		targetV_L = avgV - avgV * (self.Length/2) *k

		return targetV_R,targetV_L

	def createPointMarker2(self, points, marker_id, rgba = None, pose=[0,0,0,0,0,0,1], frame_id = '/world'):
		marker = Marker()
		marker.header.frame_id = "/world"
		marker.type = marker.POINTS
		marker.scale.x = 0.01
		marker.scale.y = 0.01
		marker.scale.z = 0.01
		marker.id = marker_id
		n = len(points)
		print "n = ",n
		sub = 1
	
		if rgba is not None:
			marker.color.r, marker.color.g, marker.color.b, marker.color.a = tuple(rgba)

		for i in xrange(0,n,sub):
			p = Point()
			p.x = points[i][0]
			p.y = points[i][1]
			p.z = points[i][2]
			marker.points.append(p)


		if rgba is None:
			for i in xrange(0,n,sub):
				p = ColorRGBA()
				p.r = points[i][3]
				p.g = points[i][4]
				p.b = points[i][5]
				p.a = points[i][6]
				marker.colors.append(p)

		# marker.pose = poselist2pose(pose)

		return marker
	def publish_marker(self):
		points = []	
		for i in np.linspace(0, 1):
			points.append([i, 0, 0])
		for i in np.linspace(-np.pi/2, np.pi/2):
			points.append([np.cos(i)*0.25 + 1, np.sin(i)*0.25 + 0.25, 0])
		for i in np.linspace(0, 1):
			points.append([i, 0.5, 0])

		for i in xrange(0,9):
			self.vis_pub.publish(self.createPointMarker2(points, 1, [0.6, 0.6, 0, 1]))

if __name__ == "__main__":
	rospy.init_node("mitlab2_node",anonymous=False)
	mitlab2_node = mitlab2_node()
	rospy.spin()

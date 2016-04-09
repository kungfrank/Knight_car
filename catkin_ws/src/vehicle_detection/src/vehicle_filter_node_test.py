#!/usr/bin/env python
from cv_bridge import CvBridge, CvBridgeError
from duckietown_msgs.msg import VehicleCorners, VehiclePose
from geometry_msgs.msg import Point32
import rospy
import cv2
import io
import numpy as np
import threading

class VehicleFilterNodeTest(object):
	def __init__(self):
		self.node_name = "Vehicle Filter Test"
		self.bridge = CvBridge()
		self.sub_pose = rospy.Subscriber("~pose", VehiclePose, 
				self.cbPose, queue_size=1)
		self.pub_corners = rospy.Publisher("~corners", VehicleCorners, queue_size=1)
		
		rospy.loginfo("Initialization of [%s] completed" % (self.node_name))
		pub_period = rospy.get_param("~pub_period", 1.0)
		rospy.Timer(rospy.Duration.from_sec(pub_period), self.pubCorners)

	def cbPose(self, pose_msg):
		# Start a daemon thread to process the image
		thread = threading.Thread(target=self.processPose,args=(pose_msg,))
		thread.setDaemon(True)
		thread.start()
		# Returns rightaway

	def pubCorners(self, args=None):
		corners_msg_out = VehicleCorners()
		corners_msg_out.header.stamp = rospy.Time.now()
		p_1 = Point32()
		p_2 = Point32()
		p_3 = Point32()

		p_1.x = 0.0
		p_1.y = 10.0

		p_2.x = 1.0
		p_2.y = 11.0

		p_3.x = 2.0
		p_3.y = 12.0

		corners_msg_out.corners.append(p_1)
		corners_msg_out.corners.append(p_2)
		corners_msg_out.corners.append(p_3)
		self.pub_corners.publish(corners_msg_out)

		rospy.loginfo("Publishing corners")
		

	def processPose(self, pose_msg):
		rospy.loginfo('Pose received : (rho = %.2f, theta = %.2f, psi = %.2f)' %
					(pose_msg.rho.data, pose_msg.theta.data, pose_msg.psi.data))

if __name__ == '__main__': 
	rospy.init_node('vehicle_filter_node', anonymous=False)
	vehicle_filter_test_node = VehicleFilterNodeTest()
	rospy.spin()


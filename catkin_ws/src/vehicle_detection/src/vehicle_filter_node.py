#!/usr/bin/env python
from cv_bridge import CvBridge, CvBridgeError
from duckietown_msgs.msg import VehicleCorners, VehiclePose
from geometry_msgs.msg import Point32
import rospy
import cv2
import io
import numpy as np
import threading

class VehicleFilterNode(object):
	def __init__(self):
		self.node_name = "Vehicle Filter"
		self.bridge = CvBridge()
		self.sub_corners = rospy.Subscriber("~corners", VehicleCorners, 
				self.cbCorners, queue_size=1)
		self.pub_pose = rospy.Publisher("~pose", VehiclePose, queue_size=1)
		rospy.loginfo("Initialization of [%s] completed" % (self.node_name))

	def cbCorners(self, vehicle_corners_msg):
		# Start a daemon thread to process the image
		thread = threading.Thread(target=self.processCorners,args=(vehicle_corners_msg,))
		thread.setDaemon(True)
		thread.start()
		# Returns rightaway

	def processCorners(self, vehicle_corners_msg):
		for i in np.arange(len(vehicle_corners_msg.corners)):
			rospy.loginfo('Corners received : (x = %.2f, y = %.2f)' %
					(vehicle_corners_msg.corners[i].x, vehicle_corners_msg.corners[i].y))

		pose_msg_out = VehiclePose()

		pose_msg_out.rho.data = 1.2
		pose_msg_out.theta.data = 2.3
		pose_msg_out.psi.data = 3.4

		self.pub_pose.publish(pose_msg_out)

if __name__ == '__main__': 
	rospy.init_node('vehicle_filter_node', anonymous=False)
	vehicle_filter_node = VehicleFilterNode()
	rospy.spin()

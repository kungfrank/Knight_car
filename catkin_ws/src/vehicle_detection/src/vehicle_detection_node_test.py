#!/usr/bin/env python
from cv_bridge import CvBridge, CvBridgeError
from duckietown_msgs.msg import VehicleCorners
from geometry_msgs.msg import Point32
from sensor_msgs.msg import CompressedImage, Image
import cv2
import io
import numpy as np
import rospy
import threading

class VehicleDetectionTestNode(object):
	def __init__(self):
		self.node_name = "Vehcile Detection Test"
		self.bridge = CvBridge()
		self.pub_image = rospy.Publisher("~image", CompressedImage, queue_size=1)
		self.sub_image = rospy.Subscriber("~corners", VehicleCorners, 
				self.cbCorners, queue_size=1)

		self.original_filename = rospy.get_param('~original_image_file')
		self.original_image = cv2.imread(self.original_filename)
	
		rospy.loginfo("Initialization of [%s] completed" % (self.node_name))
		pub_period = rospy.get_param("~pub_period", 1.0)
		rospy.Timer(rospy.Duration.from_sec(pub_period), self.pubOrig)

	def cbCorners(self,corners_msg):
		# Start a daemon thread to process the image
		thread = threading.Thread(target=self.processCorners,args=(corners_msg,))
		thread.setDaemon(True)
		thread.start()
		# Returns rightaway

	def pubOrig(self, args=None):
		image_msg_out = CompressedImage()
		image_msg_out.header.stamp = rospy.Time.now()
		image_msg_out.format = "png"
		image_msg_out.data = np.array(cv2.imencode('.png',
				self.original_image)[1]).tostring()

		self.pub_image.publish(image_msg_out)
		rospy.loginfo("Publishing original image")
		

	def processCorners(self, corners_msg):
		for i in np.arange(len(corners_msg.corners)):
			rospy.loginfo('Corners received : (x = %.2f, y = %.2f)' %
					(corners_msg.corners[i].x, corners_msg.corners[i].y))


if __name__ == '__main__': 
	rospy.init_node('vehicle_detection_test_node', anonymous=False)
	virtual_mirror_test_node = VehicleDetectionTestNode()
	rospy.spin()



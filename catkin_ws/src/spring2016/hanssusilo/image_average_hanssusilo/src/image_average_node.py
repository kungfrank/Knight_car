#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class ImageAverager:
	def __init__(self):	
		# Create publisher
		self.publisher = rospy.Publisher("~img_topic_out",Image,queue_size=1)
		# Create subscriber
		self.subscriber = rospy.Subscriber("~img_topic_in",CompressedImage,self.callback)
		# Create bridge
		self.bridge = CvBridge()

		# Initialize variables for averaged image
		self.avgimg = None
		self.N = 0.0
		self.test = 0
	
	# Define callback function
	def callback(self,msg):
		imgcv = cv2.imdecode(np.fromstring(msg.data, np.uint8), cv2.CV_LOAD_IMAGE_COLOR)
		if self.N>0:
			self.avgimg = self.avgimg*self.N/(self.N+1)+imgcv/(self.N+1)
		else:
			self.avgimg = imgcv
		self.N = self.N+1
		imgout = self.bridge.cv2_to_imgmsg(np.rint(self.avgimg).astype(np.uint8), "bgr8")
		imgout.header.stamp = msg.header.stamp
		self.publisher.publish(imgout)
	
if __name__ == '__main__':
	# Initialize the node with rospy
	rospy.init_node('image_average_node')
	# Initialize node class
	ia = ImageAverager()
	rospy.spin() #Keeps the script from exiting

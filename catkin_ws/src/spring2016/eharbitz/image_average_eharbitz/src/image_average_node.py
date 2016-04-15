#!/usr/bin/env python
import rospy

import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import cv2
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage

class image_average():

	def __init__(self):

		self.bridge = CvBridge()
		self.numIm = 0

		# Create publisher
		self.publisher = rospy.Publisher("~image_out", Image, queue_size=1)

		# Create subscriber
		self.subscriber = rospy.Subscriber("~image_in", CompressedImage, self.callback, queue_size=1)
		
		self.averageImage = None

	# Define Timer callback
	def callback(self, ros_data):
		# Convert to CV2
		np_arr = np.fromstring(ros_data.data, np.uint8)
		newImage_u8 = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
		newImage = newImage_u8.astype('float32')
		rgb_out_32 = newImage

		# We need to calculate an average. I use a comulative moving average.
		alpha = self.numIm/(self.numIm+1.0)
		beta = 1.0/(self.numIm+1)

		if self.numIm == 0:
			self.averageImage = newImage # Avoid problem with type None
		
		cv2.addWeighted(self.averageImage, alpha, newImage, beta, 0, rgb_out_32)
		self.numIm += 1

		self.averageImage = rgb_out_32

		imageOut = rgb_out_32.astype('uint8')

		# Back to ROS format(?)
		msg = self.bridge.cv2_to_imgmsg(imageOut,"bgr8")
		self.publisher.publish(msg)
	
if __name__ == '__main__':
	# Initialize the node with rospy
	rospy.init_node('image_average_node')

	ia = image_average()
	
	# spin to keep the script for exiting
	rospy.spin()
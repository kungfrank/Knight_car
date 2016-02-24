#!/usr/bin/env python
from copy import deepcopy
from cv_bridge import CvBridge, CvBridgeError
from mutex import mutex
from sensor_msgs.msg import CompressedImage, Image
import cv2
import io
import numpy as np
import rospy
import threading

class ImageAverageNode(object):
	def __init__(self):
		self.node_name = "Image Average"
		self.bridge = CvBridge()
		self.pub_image = rospy.Publisher("~average_image", Image, queue_size=1)
		self.sub_image = rospy.Subscriber("/ayrton/camera_node/image/compressed", CompressedImage, 
				self.cbImage, queue_size=1)
		rospy.loginfo("Initialization of [%s] completed" % (self.node_name))
		self.lock = mutex()
		self.lock.unlock()
		self.old_img = None

	def cbImage(self,image_msg):
		# Start a daemon thread to process the image
		thread = threading.Thread(target=self.processImage,args=(image_msg,))
		thread.setDaemon(True)
		thread.start()
		# Returns rightaway
	def processImage(self, image_msg):
		if self.lock.testandset():
			image_cv = cv2.imdecode(np.fromstring(image_msg.data, np.uint8), cv2.CV_LOAD_IMAGE_COLOR)
			if self.old_img is not None:
				avg_img = cv2.addWeighted(image_cv, 0.5, self.old_img, 0.5, 0.0)
				self.old_img = deepcopy(image_cv)
				rospy.loginfo("Image Shape: [%d x %d]." % (image_cv.shape[0], image_cv.shape[1]))
				image_msg_out = self.bridge.cv2_to_imgmsg(avg_img, "bgr8")
				image_msg_out.header.stamp = image_msg.header.stamp
				self.pub_image.publish(image_msg_out)
			else:
				self.old_img = deepcopy(image_cv)
			self.lock.unlock()

if __name__ == '__main__': 
	rospy.init_node('virtual_mirror', anonymous=False)
	image_average_node = ImageAverageNode()
	rospy.spin()

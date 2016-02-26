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
		self.sub_image = rospy.Subscriber("/ferrari/camera_node/image/compressed", CompressedImage, 
				self.cbImage, queue_size=1)
		rospy.loginfo("Initialization of [%s] completed" % (self.node_name))
		self.lock = mutex()
		self.lock.unlock()
		self.avg_img = None
		self.count_images = 0

	def cbImage(self,image_msg):
		# Start a daemon thread to process the image
		thread = threading.Thread(target=self.processImage,args=(image_msg,))
		thread.setDaemon(True)
		thread.start()
		# Returns rightaway
	def processImage(self, image_msg):
		if self.lock.testandset():
			image_cv = cv2.imdecode(np.fromstring(image_msg.data, np.uint8), cv2.CV_LOAD_IMAGE_COLOR)
			if self.avg_img is not None:
				img_float = cv2.normalize(image_cv.astype('float'), None, 0.0, 255.0, cv2.NORM_MINMAX)
				alpha = 1 / float(self.count_images + 1)
				beta  = float(self.count_images) / float(self.count_images + 1)
				self.avg_img = cv2.addWeighted(img_float, alpha, self.avg_img, beta, 0.0)
				img_out = np.uint8(deepcopy(self.avg_img))
				rospy.loginfo("Image Shape: [%d x %d]. 1 / (n + 1) = %1.4f" % (img_out.shape[0], img_out.shape[1], alpha))
				image_msg_out = self.bridge.cv2_to_imgmsg(img_out, "bgr8")
				image_msg_out.header.stamp = image_msg.header.stamp
				self.pub_image.publish(image_msg_out)
				self.count_images += 1
			else:
				self.avg_img = cv2.normalize(image_cv.astype('float'), None, 0.0, 1.0, cv2.NORM_MINMAX)
				self.count_images += 1
			self.lock.unlock()

if __name__ == '__main__': 
	rospy.init_node('virtual_mirror', anonymous=False)
	image_average_node = ImageAverageNode()
	rospy.spin()

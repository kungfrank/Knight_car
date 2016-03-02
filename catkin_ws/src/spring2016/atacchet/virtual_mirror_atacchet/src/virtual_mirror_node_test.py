#!/usr/bin/env python
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image
import rospy
import cv2
import io
import numpy as np
import threading

class VirtualMirrorTestNode(object):
	def __init__(self):
		self.node_name = "Virtual Mirror Test"
		self.bridge = CvBridge()
		self.pub_image = rospy.Publisher("~original_image", Image, queue_size=1)
		self.sub_image = rospy.Subscriber("~processed_image", CompressedImage, 
				self.cbImage, queue_size=1)
		self.original_filename = rospy.get_param('~original_image_file')
		self.original_image = cv2.imread(self.original_filename)
		self.flipped_image  = cv2.flip(self.original_image, 1)
		self.fplipped_data = np.asarray(self.flipped_image, dtype=np.uint8)
		rospy.loginfo("Initialization of [%s] completed" % (self.node_name))
		pub_period = rospy.get_param("~pub_period", 1.0)
		rospy.Timer(rospy.Duration.from_sec(pub_period), self.pubOrig)
		

	def cbImage(self,image_msg):
		# Start a daemon thread to process the image
		thread = threading.Thread(target=self.processImage,args=(image_msg,))
		thread.setDaemon(True)
		thread.start()
		# Returns rightaway

	def pubOrig(self):
		image_msg_out = self.bridge.cv2_to_imgmsg(self.flipped_image, "bgr8")
		image_msg_out.header.stamp = image_msg.header.stamp
		self.pub_image.publish(image_msg_out)
		rospy.loginfo("Publishing original image")
		

	def processImage(self, image_msg):
		image_cv = cv2.imdecode(np.fromstring(image_msg.data, np.uint8), cv2.CV_LOAD_IMAGE_COLOR)
		image_data = np.asarray(image_cv, dtype=np.uint8)
		data_check = self.flipped_data - image_data
		rospy.loginfo("Received flipped image :: sum is %d" % np.sum(data_check))


if __name__ == '__main__': 
	rospy.init_node('virtual_mirror_atacchet_test', anonymous=False)
	virtual_mirror_test_node = VirtualMirrorTestNode()
	rospy.spin()


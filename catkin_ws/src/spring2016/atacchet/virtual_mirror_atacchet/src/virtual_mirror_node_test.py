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
		self.pub_image = rospy.Publisher("~original_image", CompressedImage, queue_size=1)
		self.sub_image = rospy.Subscriber("~processed_image", Image, 
				self.cbImage, queue_size=1)
		self.original_filename = rospy.get_param('~original_image_file')
		self.original_image = cv2.imread(self.original_filename)
		self.horizontal_filename = rospy.get_param('~horizontal_image_file')
		self.vertical_filename = rospy.get_param('~vertical_image_file')
	
		self.flipped_h_img = cv2.imread(self.horizontal_filename)
		self.flipped_v_img = cv2.imread(self.vertical_filename)
		self.flipped_data_h = np.asarray(self.flipped_h_img, dtype=np.uint8)
		self.flipped_data_v = np.asarray(self.flipped_v_img, dtype=np.uint8)

		rospy.loginfo("Initialization of [%s] completed" % (self.node_name))
		pub_period = rospy.get_param("~pub_period", 1.0)
		rospy.Timer(rospy.Duration.from_sec(pub_period), self.pubOrig)

	def cbImage(self,image_msg):
		# Start a daemon thread to process the image
		thread = threading.Thread(target=self.processImage,args=(image_msg,))
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
		

	def processImage(self, image_msg):
		image_cv = self.bridge.imgmsg_to_cv2(image_msg)
		rospy.loginfo("test image size = [%d x %d]." % (image_cv.shape[0],
				image_cv.shape[1]))
		image_data = np.asarray(image_cv, dtype=np.uint8)
		data_check_h = self.flipped_data_h - image_data
		data_check_v = self.flipped_data_v - image_data

		if np.sum(data_check_h) < 1:
			rospy.loginfo("Pass H:: sum is %d" % np.sum(data_check_h))
			return
		if np.sum(data_check_v) < 1:
			rospy.loginfo("Pass V:: sum is %d" % np.sum(data_check_v))
			return
		rospy.loginfo("Fail :: sum_h is %d and sum_v is %d" % 
				(np.sum(data_check_h), np.sum(data_check_v)))


if __name__ == '__main__': 
	rospy.init_node('virtual_mirror_atacchet_test', anonymous=False)
	virtual_mirror_test_node = VirtualMirrorTestNode()
	rospy.spin()


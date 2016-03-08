#!/usr/bin/env python
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image
from duckietown_msgs_atacchet.msg import HorzVert
import rospy
import cv2
import io
import numpy as np
import threading

class VirtualMirrorNode(object):
	def __init__(self):
		self.node_name = "Virtual Mirror"
		self.bridge = CvBridge()
		self.flip_dir = HorzVert.HORZ


		self.pub_image = rospy.Publisher("~mirror_image", Image, queue_size=1)
		self.sub_image = rospy.Subscriber("~incoming_image", CompressedImage, 
				self.cbImage, queue_size=1)
		rospy.loginfo("Initialization of [%s] completed" % (self.node_name))

		self.param_timer = rospy.Timer(rospy.Duration.from_sec(1.0), 
				self.cbParamTimer)

		self.pub_config = rospy.Publisher("~flip_param_config", HorzVert,
				queue_size=1)
		self.config_pub_timer = rospy.Timer(rospy.Duration.from_sec(1.0),
				self.cbPubConfig)

	def cbParamTimer(self, event):
		self.flip_dir = rospy.get_param("~flip_direction")

	def cbPubConfig(self, event):
		flip_param_msg = HorzVert()
		flip_param_msg.flip_dir = self.flip_dir
		self.pub_config.publish(flip_param_msg)

	def cbImage(self,image_msg):
		# Start a daemon thread to process the image
		thread = threading.Thread(target=self.processImage,args=(image_msg,))
		thread.setDaemon(True)
		thread.start()
		# Returns rightaway

	def processImage(self, image_msg):
		image_cv = cv2.imdecode(np.fromstring(image_msg.data, np.uint8), cv2.CV_LOAD_IMAGE_COLOR)
		fimage = cv2.flip(image_cv, self.flip_dir)
		rospy.loginfo("Image Shape: [%d x %d]." % (image_cv.shape[0], image_cv.shape[1]))
		image_msg_out = self.bridge.cv2_to_imgmsg(fimage, "bgr8")
		image_msg_out.header.stamp = image_msg.header.stamp
		self.pub_image.publish(image_msg_out)

if __name__ == '__main__': 
	rospy.init_node('virtual_mirror', anonymous=False)
	virtual_mirror_node = VirtualMirrorNode()
	rospy.spin()

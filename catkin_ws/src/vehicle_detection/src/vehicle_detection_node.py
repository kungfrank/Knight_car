#!/usr/bin/env python
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image
from duckietown_msgs.msg import VehicleCorners
from geometry_msgs.msg import Point32
import rospy
import cv2
import io
import numpy as np
import threading

class VehicleDetectionNode(object):
	def __init__(self):
		self.node_name = "Vehicle Detection"
		self.bridge = CvBridge()
		self.sub_image = rospy.Subscriber("~image", CompressedImage, 
				self.cbImage, queue_size=1)
		self.pub_corners = rospy.Publisher("~corners", VehicleCorners, queue_size=1)
		rospy.loginfo("Initialization of [%s] completed" % (self.node_name))

	def cbImage(self, image_msg):
		# Start a daemon thread to process the image
		thread = threading.Thread(target=self.processImage,args=(image_msg,))
		thread.setDaemon(True)
		thread.start()
		# Returns rightaway

	def processImage(self, image_msg):
		image_cv = cv2.imdecode(np.fromstring(image_msg.data, np.uint8), cv2.CV_LOAD_IMAGE_COLOR)
		rospy.loginfo("Image Shape: [%d x %d]." % (image_cv.shape[0], image_cv.shape[1]))

		corners_msg_out = VehicleCorners()
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

if __name__ == '__main__': 
	rospy.init_node('vehicle_detection', anonymous=False)
	vehicle_detection_node = VehicleDetectionNode()
	rospy.spin()


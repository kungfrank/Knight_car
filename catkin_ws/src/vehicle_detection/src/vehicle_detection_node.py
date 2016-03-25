#!/usr/bin/env python
from copy import deepcopy
from cv_bridge import CvBridge, CvBridgeError
from duckietown_msgs.msg import VehicleCorners
from geometry_msgs.msg import Point32
from mutex import mutex
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
import cv2
import io
import numpy as np
import rospy
import stopit
import threading

class VehicleDetectionNode(object):
	def __init__(self):
		self.node_name = "Vehicle Detection"
		self.bridge = CvBridge()
		self.sub_image = rospy.Subscriber("~image", CompressedImage, 
				self.cbImage, queue_size=1)
		self.pub_corners = rospy.Publisher("~corners", 
				VehicleCorners, queue_size=1)
		self.pub_chessboard_image = rospy.Publisher("~chessboard_image", 
				Image, queue_size=1)
		self.lock = mutex()
		rospy.loginfo("Initialization of [%s] completed" % (self.node_name))
	
	def cbImage(self, image_msg):
		# Start a daemon thread to process the image
		thread = threading.Thread(target=self.processImage,args=(image_msg,))
		thread.setDaemon(True)
		thread.start()
		# Returns rightaway
	
	def processImage(self, image_msg):
		if self.lock.testandset():
			corners_msg_out = VehicleCorners()
			image_cv = cv2.imdecode(np.fromstring(image_msg.data, np.uint8), 
					cv2.CV_LOAD_IMAGE_COLOR)
			rospy.loginfo("Image Shape: [%d x %d]." % 
					(image_cv.shape[0], image_cv.shape[1]))
			with stopit.ThreadingTimeout(0.2) as to_ctx_mgr:
				start = rospy.Time.now()
				(detection, corners) = cv2.findChessboardCorners(image_cv, 
						(5, 7))
				rospy.loginfo("find chessboard corners took: %.3f", 
						(rospy.Time.now() - start).to_sec())
			if not to_ctx_mgr:
				rospy.loginfo("find chessboard corners took too long")
				corners_msg_out.detection = False
				self.pub_corners.publish(corners_msg_out)
				self.lock.unlock()
				return
			cv2.drawChessboardCorners(image_cv, (5, 7), corners, detection)
			image_msg_out = self.bridge.cv2_to_imgmsg(image_cv, "bgr8")
			self.pub_chessboard_image.publish(image_msg_out)
			if not detection:
				corners_msg_out.detection = False
				self.pub_corners.publish(corners_msg_out)
				self.lock.unlock()
				rospy.loginfo("Corners not found")
				return
			# publish debug chessboard image
			rospy.loginfo("Corners FOUND")
			corners_msg_out.detection = True
			for i in np.arange(corners.shape[0]):
				p = Point32()
				p.x = corners[i][0][0]
				p.y = corners[i][0][1]
				corners_msg_out.corners.append(deepcopy(p))
			self.pub_corners.publish(corners_msg_out)
			self.lock.unlock()

if __name__ == '__main__': 
	rospy.init_node('vehicle_detection', anonymous=False)
	vehicle_detection_node = VehicleDetectionNode()
	rospy.spin()


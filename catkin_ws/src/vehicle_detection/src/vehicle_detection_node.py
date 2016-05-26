#!/usr/bin/env python
from copy import deepcopy
from cv_bridge import CvBridge, CvBridgeError
from duckietown_msgs.msg import BoolStamped
from geometry_msgs.msg import Point32
from mutex import mutex
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32
import cv2
import numpy as np
import os
import rospkg
import rospy
import threading
import time
import yaml

class VehicleDetectionNode(object):

	def __init__(self):
		self.node_name = rospy.get_name()
		self.bridge = CvBridge()
		self.active = True
		self.config	= self.setupParam("~config", "baseline")
		self.cali_file_name = self.setupParam("~cali_file_name", "default")
		rospack = rospkg.RosPack()
		self.cali_file = rospack.get_path('duckietown') + \
				"/config/" + self.config + \
				"/vehicle_detection/vehicle_detection_node/" +  \
				self.cali_file_name + ".yaml"
		if not os.path.isfile(self.cali_file):
			rospy.logwarn("[%s] Can't find calibration file: %s.\n" 
					% (self.node_name, self.cali_file))
		self.loadConfig(self.cali_file)
		self.sub_image = rospy.Subscriber("~image", Image, 
				self.cbImage, queue_size=1)
		self.sub_switch = rospy.Subscriber("~switch", BoolStamped,
				self.cbSwitch, queue_size=1)
		self.pub_detection = rospy.Publisher("~detection", 
				BoolStamped, queue_size=1)
		self.pub_circlepattern_image = rospy.Publisher("~circlepattern_image", 
				Image, queue_size=1)
		self.pub_time_elapsed = rospy.Publisher("~detection_time",
			Float32, queue_size=1)
		self.lock = mutex()
		rospy.loginfo("[%s] Initialization completed" % (self.node_name))
	
	def setupParam(self,param_name,default_value):
		value = rospy.get_param(param_name,default_value)
		rospy.set_param(param_name, value)
		rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
		return value

	def loadConfig(self, filename):
		stream = file(filename, 'r')
		data = yaml.load(stream)
		stream.close()
                self.circlepattern_dims = tuple(data['circlepattern_dims']['data'])
		self.blobdetector_min_area = data['blobdetector_min_area']
		self.blobdetector_min_dist_between_blobs = data['blobdetector_min_dist_between_blobs']
		self.publish_circles = data['publish_circles']
                rospy.loginfo('[%s] circlepattern_dim : %s' % (self.node_name, 
                               	self.circlepattern_dims,))
		rospy.loginfo('[%s] blobdetector_min_area: %.2f' % (self.node_name, 
				self.blobdetector_min_area))
		rospy.loginfo('[%s] blobdetector_min_dist_between_blobs: %.2f' % (self.node_name, 
				self.blobdetector_min_dist_between_blobs))
		rospy.loginfo('[%s] publish_circles: %r' % (self.node_name, 
				self.publish_circles))

	def cbSwitch(self, switch_msg):
		self.active = switch_msg.data

	def cbImage(self, image_msg):
		if not self.active:
			return
		# Start a daemon thread to process the image
		thread = threading.Thread(target=self.processImage,args=(image_msg,))
		thread.setDaemon(True)
		thread.start()
		# Returns rightaway
	
	def processImage(self, image_msg):
		if self.lock.testandset():
			vehicle_detected_msg_out = BoolStamped()
			try:
				image_cv=self.bridge.imgmsg_to_cv2(image_msg,"bgr8")
			except CvBridgeError as e:
				print e
			start = rospy.Time.now()
			params = cv2.SimpleBlobDetector_Params()
			params.minArea = self.blobdetector_min_area
			params.minDistBetweenBlobs = self.blobdetector_min_dist_between_blobs
			simple_blob_detector = cv2.SimpleBlobDetector(params)
			(detection, corners) = cv2.findCirclesGrid(image_cv,
					self.circlepattern_dims, flags=cv2.CALIB_CB_SYMMETRIC_GRID,
					blobDetector=simple_blob_detector)
			elapsed_time = (rospy.Time.now() - start).to_sec()
			self.pub_time_elapsed.publish(elapsed_time)
			vehicle_detected_msg_out.data = detection
			self.pub_detection.publish(vehicle_detected_msg_out)
			if self.publish_circles:
				cv2.drawChessboardCorners(image_cv, 
						self.circlepattern_dims, corners, detection)
				image_msg_out = self.bridge.cv2_to_imgmsg(image_cv, "bgr8")
				self.pub_circlepattern_image.publish(image_msg_out)
			self.lock.unlock()

if __name__ == '__main__': 
	rospy.init_node('vehicle_detection', anonymous=False)
	vehicle_detection_node = VehicleDetectionNode()
	rospy.spin()


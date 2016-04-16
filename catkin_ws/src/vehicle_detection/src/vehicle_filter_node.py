#!/usr/bin/env python
from cv_bridge import CvBridge, CvBridgeError
from duckietown_msgs.msg import VehicleCorners, VehiclePose
from geometry_msgs.msg import Point32
from image_geometry import PinholeCameraModel
from mutex import mutex
from sensor_msgs.msg import CameraInfo
import cv2
import numpy as np
import os
import rospkg
import rospy
import threading
import yaml

class VehicleFilterNode(object):
	def __init__(self):
		self.node_name = rospy.get_name()
		self.bridge = CvBridge()

		self.config	= self.setupParam("~config", "baseline")
		self.cali_file_name = self.setupParam("~cali_file_name", "default")
		rospack = rospkg.RosPack()
		self.cali_file = rospack.get_path('duckietown') + \
				"/config/" + self.config + \
				"/vehicle_detection/vehicle_filter_node/" +  \
				self.cali_file_name + ".yaml"
		if not os.path.isfile(self.cali_file):
			rospy.logwarn("[%s] Can't find calibration file: %s.\n" 
					% (self.node_name, self.cali_file))
		self.loadConfig(self.cali_file)
		self.sub_corners = rospy.Subscriber("~corners", VehicleCorners, 
				self.cbCorners, queue_size=1)
		self.pub_pose = rospy.Publisher("~pose", VehiclePose, queue_size=1)
		self.sub_info = rospy.Subscriber("~camera_info", CameraInfo,
				self.cbCameraInfo, queue_size=1)
		self.pcm = PinholeCameraModel()
		rospy.loginfo("[%s] Initialization completed" % (self.node_name))
		self.lock = mutex()

	def setupParam(self,param_name,default_value):
		value = rospy.get_param(param_name,default_value)
		rospy.set_param(param_name, value)
		rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
		return value

	def loadConfig(self, filename):
		stream = file(filename, 'r')
		data = yaml.load(stream)
		stream.close()
		self.distance_between_centers = data['distance_between_centers']
		rospy.loginfo('[%s] distance_between_centers dim : %s' % (self.node_name, 
				self.distance_between_centers))

	def cbCameraInfo(self, camera_info_msg):
		thread = threading.Thread(target=self.processCameraInfo,
				args=(camera_info_msg,))
		thread.setDaemon(True)
		thread.start()
	
	def processCameraInfo(self, camera_info_msg):
		if self.lock.testandset():
			self.pcm.fromCameraInfo(camera_info_msg)
			self.lock.unlock()

	def cbCorners(self, vehicle_corners_msg):
		# Start a daemon thread to process the image
		thread = threading.Thread(target=self.processCorners,
				args=(vehicle_corners_msg,))
		thread.setDaemon(True)
		thread.start()
		# Returns rightaway

	def processCorners(self, vehicle_corners_msg):
		# do nothing - just relay the detection
		if self.lock.testandset():
				pose_msg_out = VehiclePose()
				pose_msg_out.rho.data = 0.0
				pose_msg_out.theta.data = 0.0
				pose_msg_out.psi.data = 0.0
				pose_msg_out.detection.data = \
						vehicle_corners_msg.detection.data
				self.pub_pose.publish(pose_msg_out)
				self.lock.unlock()
				return

if __name__ == '__main__': 
	rospy.init_node('vehicle_filter_node', anonymous=False)
	vehicle_filter_node = VehicleFilterNode()
	rospy.spin()

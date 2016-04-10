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
		self.node_name = "Vehicle Filter"
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
		if self.lock.testandset():
			if not vehicle_corners_msg.detection.data:
				self.lock.unlock()
				pose_msg_out = VehiclePose()
				pose_msg_out.rho.data = 0.0
				pose_msg_out.theta.data = 0.0
				pose_msg_out.psi.data = 0.0
				pose_msg_out.detection.data = False
				self.pub_pose.publish(pose_msg_out)
				return
			height = vehicle_corners_msg.H 
			width  = vehicle_corners_msg.W
			unit_length = self.distance_between_centers

			object_corners = np.ones((height * width, 2), dtype=np.float32)
			for i in np.arange(len(vehicle_corners_msg.corners)):
				object_corners[i][0] = \
						np.float32(vehicle_corners_msg.corners[i].x)
				object_corners[i][1] = \
						np.float32(vehicle_corners_msg.corners[i].y)
			distCoeff = np.float32(self.pcm.distortionCoeffs())
			K = np.float32(self.pcm.fullIntrinsicMatrix())
			objp = np.zeros((height * width, 3), np.float32)
			objp[:, :2] = np.mgrid[0:height, 0:width].T.reshape(-1, 2)
			objp = objp * unit_length
			rvecs, tvecs, inliers = cv2.solvePnPRansac(objp, object_corners, 
					K, distCoeff)
			xrot, yrot, zrot = rvecs
			psi = zrot
			rho = np.linalg.norm(tvecs)
			theta = 0.0
			pose_msg_out = VehiclePose()
			pose_msg_out.rho.data = rho
			pose_msg_out.theta.data = theta
			pose_msg_out.psi.data = psi
			pose_msg_out.detection.data = True
			self.pub_pose.publish(pose_msg_out)
			self.lock.unlock()

if __name__ == '__main__': 
	rospy.init_node('vehicle_filter_node', anonymous=False)
	vehicle_filter_node = VehicleFilterNode()
	rospy.spin()

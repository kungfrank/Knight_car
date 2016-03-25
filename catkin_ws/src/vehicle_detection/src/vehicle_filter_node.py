#!/usr/bin/env python
from cv_bridge import CvBridge, CvBridgeError
from duckietown_msgs.msg import VehicleCorners, VehiclePose
from image_geometry import PinholeCameraModel
from mutex import mutex
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Point32
import rospy
import cv2
import io
import numpy as np
import threading

class VehicleFilterNode(object):
	def __init__(self):
		self.node_name = "Vehicle Filter"
		self.bridge = CvBridge()
		self.sub_corners = rospy.Subscriber("~corners", VehicleCorners, 
				self.cbCorners, queue_size=1)
		self.pub_pose = rospy.Publisher("~pose", VehiclePose, queue_size=1)
		self.sub_info = rospy.Subscriber("~camera_info", CameraInfo,
				self.cbCameraInfo, queue_size=1)
		self.pcm = PinholeCameraModel()
		rospy.loginfo("Initialization of [%s] completed" % (self.node_name))
		self.lock = mutex()

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
		thread = threading.Thread(target=self.processCorners,args=(vehicle_corners_msg,))
		thread.setDaemon(True)
		thread.start()
		# Returns rightaway

	def processCorners(self, vehicle_corners_msg):
		#Test node output
		#for i in np.arange(len(vehicle_corners_msg.corners)):
		#	rospy.loginfo('Corners received : (x = %.2f, y = %.2f)' %
		#			(vehicle_corners_msg.corners[i].x, vehicle_corners_msg.corners[i].y))
		if self.lock.testandset():
			if not vehicle_corners_msg.detection:
				self.lock.unlock()
				return
			height = 5 #height of checkerboard in blocks
			width = 7 #width of checkerboard in blocks
			unit_length = 12.5 #length of block in mm

			object_corners = np.ones((height * width,2), dtype=np.float32)
			for i in np.arange(len(vehicle_corners_msg.corners)):
				object_corners[i][0] = \
						np.float32(vehicle_corners_msg.corners[i].x)
				object_corners[i][1] = \
					np.float32(vehicle_corners_msg.corners[i].y)
			

			distCoeff = np.float32(self.pcm.distortionCoeffs())
			K = self.pcm.fullIntrinsicMatrix()
			K = np.float32(K)
			rospy.loginfo("K.shape: (%d, %d)" % K.shape)

			objp = np.zeros((5*6,3), np.float32)
			objp[:,:2] = np.mgrid[0:5,0:6].T.reshape(-1,2)
			objp = objp/unit_length

			rvecs, tvecs, inliers = cv2.solvePnPRansac(objp, object_corners, K, distCoeff)

			#Need to confirm that these are correct (not sure which is psi)
			xrot = rvecs[0]
			yrot = rvecs[1]
			zrot = rvecs[2]

			psi = zrot

			#If pitch and rotation are outside of some range, we can return psi = 0 to stop the car since an incorrect detection has occured

			#Calculating the length of the total displacement
			rho = np.power((np.power(tvecs[0],2)+np.power(tvecs[1],2)+np.power(tvecs[2],2)),0.5)

			#Need to calculate theta in degrees given knowledge of the horizontal translation and distance
			theta = np.arcsin(np.pi/2.) * 180. / np.pi 

			pose_msg_out = VehiclePose()

			pose_msg_out.rho.data = rho
			pose_msg_out.theta.data = theta
			pose_msg_out.psi.data = psi

			self.pub_pose.publish(pose_msg_out)
			self.lock.unlock()

if __name__ == '__main__': 
	rospy.init_node('vehicle_filter_node', anonymous=False)
	vehicle_filter_node = VehicleFilterNode()
	rospy.spin()

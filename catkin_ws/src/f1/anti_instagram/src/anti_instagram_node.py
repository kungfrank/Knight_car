#!/usr/bin/env python
import rospy
from copy import deepcopy
from sensor_msgs.msg import CompressedImage,Image
from duckietown_msgs.msg import AntiInstagramHealth
from anti_instagram.AntiInstagram import *
import numpy as np
import cv2
from cv_bridge import CvBridge,CvBridgeError

class AntiInstagramNode():
	def __init__(self):
		self.node_name = rospy.get_name()

		# Initialize publishers and subscribers
		self.pub_image = rospy.Publisher("~corrected_image",Image,queue_size=1)
		self.pub_health = rospy.Publisher("~health",AntiInstagramHealth,queue_size=1)
		self.sub_image = rospy.Subscriber("~uncorrected_image",Image,self.cbNewImage)

		# Get all params from launch file
		self.xyz = rospy.get_param("~xyz",1)

		# Initialize health message
		self.health = AntiInstagramHealth()

		self.ai = AntiInstagram()
		self.corrected_image = Image()
		self.bridge = CvBridge()

	def decodeMsg(self,image_msg):
		# rospy.loginfo(image_msg.header)
		image_cv = self.bridge.imgmsg_to_cv2(image_msg,"bgr8")
		return image_cv

	def cbNewImage(self,msg):
		'''
		Inputs:
			msg - CompressedImage - uncorrected image from raspberry pi camera
		Uses anti_instagram library to adjust msg so that it looks like the same
		color temperature as a duckietown reference image. Calculates health of the node
		and publishes the corrected image and the health state. Health somehow corresponds
		to how good of a transformation it is.
		'''
		rospy.loginfo(msg.header)
		cv_image = self.decodeMsg(msg)
		rospy.loginfo(cv_image)
		self.ai.calculateTransform(cv_image)
		corrected_image_cv2 = self.ai.applyTransform(cv_image)
		corrected_image_cv2 = np.clip(corrected_image_cv2,0,255).astype(np.uint8)
		self.corrected_image = self.bridge.cv2_to_imgmsg(corrected_image_cv2,"bgr8")

		# self.pub_health.publish(self.health)
		self.pub_image.publish(self.corrected_image)
		return

	

	def on_shutdown(self):
		rospy.loginfo("[%s] Shutting down." %(self.node_name))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('anti_instagram_node', anonymous=False)

    # Create the NodeName object
    node = AntiInstagramNode()

    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
#!/usr/bin/env python
import rospy
from copy import deepcopy
from sensor_msgs.msg import CompressedImage,Image
from duckietown_msgs.msg import AntiInstagramHealth
from anti_instagram.AntiInstagram import *
import numpy as np
import threading
import cv2
from cv_bridge import CvBridge,CvBridgeError

class AntiInstagramNode():
	def __init__(self):
		self.node_name = rospy.get_name()

		# Initialize publishers and subscribers
		self.pub_image = rospy.Publisher("~corrected_image",Image,queue_size=1)
		self.pub_health = rospy.Publisher("~health",AntiInstagramHealth,queue_size=1)
		self.sub_image = rospy.Subscriber("~uncorrected_image",Image,self.cbNewImage,queue_size=1)

		# image callback thread lock
		self.thread_lock = threading.Lock()

		# Verbose option 
		self.verbose = True #rospy.get_param('~verbose')  

		# Get all params from launch file
		self.xyz = rospy.get_param("~xyz",1)

		# Initialize health message
		self.health = AntiInstagramHealth()

		self.ai = AntiInstagram()
		self.corrected_image = Image()
		self.bridge = CvBridge()

		self.numFramesSeen = 0


	def cbNewImage(self,image_msg):
		# Start a daemon thread to process the image
		thread = threading.Thread(target=self.processImage,args=(image_msg,))
		thread.setDaemon(True)
		thread.start()
        # Returns rightaway


	def processImage(self,msg):
		'''
		Inputs:
			msg - CompressedImage - uncorrected image from raspberry pi camera
		Uses anti_instagram library to adjust msg so that it looks like the same
		color temperature as a duckietown reference image. Calculates health of the node
		and publishes the corrected image and the health state. Health somehow corresponds
		to how good of a transformation it is.
		'''

		if not self.thread_lock.acquire(False):
			# Return immediately if the thread is locked
			return

		if self.verbose:
			rospy.loginfo("[%s] Latency received = %.3f ms" %(self.node_name, (rospy.get_time()-msg.header.stamp.to_sec()) * 1000.0))
			tic = rospy.get_time()
		
		cv_image = self.bridge.imgmsg_to_cv2(msg,"bgr8")

		if self.verbose:  
			rospy.loginfo("[%s] Latency image read = %.3f ms" %(self.node_name, (rospy.get_time()-tic) * 1000.0))
			toc = rospy.get_time()
		# cv_image=cv_image*0.3 # for testing only
		# changes this to update on-line, with IIR, and not just in the beginning
		if self.numFramesSeen <= 1: #baseline param
			# only calculate transform for the first few frames
			# then apply the same transform indefintely
			self.ai.calculateTransform(cv_image)
			if self.verbose:  
				rospy.loginfo("[%s] Latency calculate transform = %.3f ms" %(self.node_name, (rospy.get_time()-toc) * 1000.0))
				toc = rospy.get_time()
			print (self.ai.scale,self.ai.shift)
		

		corrected_image_cv2 = self.ai.applyTransform(cv_image)
		if self.verbose:  
			rospy.loginfo("[%s] Latency apply transform = %.3f ms" %(self.node_name, (rospy.get_time()-toc) * 1000.0))
			toc = rospy.get_time()

		# corrected_image_cv2 = cv_image
		corrected_image_cv2 = np.clip(corrected_image_cv2, 0, 255).astype(np.uint8)
		self.corrected_image = self.bridge.cv2_to_imgmsg(corrected_image_cv2,"bgr8")

		if self.verbose:  
			rospy.loginfo("[%s] Latency image msg encode = %.3f ms" %(self.node_name, (rospy.get_time()-toc) * 1000.0))
			toc = rospy.get_time()

		# self.pub_health.publish(self.health)
		self.pub_image.publish(self.corrected_image)
		if self.verbose:  
			rospy.loginfo("[%s] Latency image publish  = %.3f ms" %(self.node_name, (rospy.get_time() - toc) * 1000.0))
			rospy.loginfo("[%s] Total Latency image published (total time frame in -> frame out) = %.3f ms" %(self.node_name, (rospy.get_time() - tic) * 1000.0))
		
		self.numFramesSeen += 1

		self.thread_lock.release()

	

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

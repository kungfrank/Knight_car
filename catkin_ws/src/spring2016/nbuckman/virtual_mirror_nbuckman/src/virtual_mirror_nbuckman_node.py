#!/usr/bin/env python
import rospy
import cv2
import duckietown_msgs.msg
from duckietown_msg_nbuckman.msg import Flip
#from virtual_mirror_nbuckman import util
from std_msgs.msg import String
import numpy as np
from sensor_msgs.msg import CompressedImage,Image
from cv_bridge import CvBridge, CvBridgeError
import time


# Initialize the node with rospy

class VirtualMirrorNbuckmanNode(object):
	def __init__(self):
		self.node_name = "Virtual Mirror Nbuckman"
		self.sub_image = rospy.Subscriber("/ernie/camera_node/image/compressed", CompressedImage,self.flipImageNP)
		self.pub_image = rospy.Publisher("~mirror_image/compressed", CompressedImage, queue_size=1)
		self.bridge = CvBridge()
		self.pub_msg = rospy.Publisher("~mirror_message",Flip, queue_size=1)
		#Take in a parameter.  Question: what should the default value be?
		self.flip_direction = self.setupParam("~flip_direction", "horz")

		#Implement a timer
		self.last_pub_time = rospy.Time.now()
		self.param_timer = rospy.Timer(rospy.Duration.from_sec(1.0),self.sendMessage)

	def sendMessage(self,event):
		flip_msg = Flip()
		flip_msg.orient = self.flip_direction
		self.pub_msg.publish(flip_msg)

	def setupParam(self,param_name,default_value):
		value = rospy.get_param(param_name,default_value)
		rospy.set_param(param_name,value) #Write to parameter server for transparancy
		rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
		return value

	def flipImageNP(self,msg):
		np_array = np.fromstring(msg.data, np.uint8)
		image_np = cv2.imdecode(np_array, cv2.CV_LOAD_IMAGE_COLOR)		
		#flip_arr = np.fliplr(np_arr)
		imageflip_np=cv2.flip(image_np,1)
		flip_im = CompressedImage()
		flip_im.data = np.array(cv2.imencode('.jpg', imageflip_np)[1]).tostring()
		#flip_im.data = flip_arr.tostring()
		flip_im.header.stamp = msg.header.stamp
		self.pub_image.publish(flip_im)


	def flipImage(self,msg):
        # Decode from compressed image
        # with OpenCV
		cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

		#image_cv = cv2.imdecode(np.fromstring(msg.data, np.uint8), cv2.CV_LOAD_IMAGE_COLOR)
		hei_original = image_cv.shape[0]
		wid_original = image_cv.shape[1]
		#reverseimage = image_cv[:,:,-1]
		reverseimg=cv2.flip(cv_image,1)
		image_msg_out = self.bridge.cv2_to_imgmsg(reverseimage, "bgr8")
		image_msg_out.header.stamp = image_msg.header.stamp
		self.pub_image.publish(image_msg_out)

		#self.pub_image.publish(flippedMsg)

		#image_cv = cv2.imdecode(np.fromstring(image_msg.data, np.uint8), cv2.CV_LOAD_IMAGE_COLOR)
		#flippedImage = image_cv



if __name__ == '__main__': 
	rospy.init_node('virtual_mirror_nbuckman')
	virtual_mirror_node = VirtualMirrorNbuckmanNode()
	rospy.spin()
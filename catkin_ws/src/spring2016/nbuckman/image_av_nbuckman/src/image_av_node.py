#!/usr/bin/env python
import rospy
import cv2
import duckietown_msgs.msg
#from virtual_mirror_nbuckman import util
from std_msgs.msg import String
import numpy as np
from sensor_msgs.msg import CompressedImage,Image
from cv_bridge import CvBridge, CvBridgeError


# Initialize the node with rospy

class ImageAvNode(object):
	def __init__(self):
		self.node_name = "Image Av"
		self.sub_image = rospy.Subscriber("/ernie/camera_node/image/compressed", CompressedImage,self.flipImageNP)
		self.pub_image = rospy.Publisher("~mirror_image/compressed", CompressedImage, queue_size=1)
		self.bridge = CvBridge()
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
	rospy.init_node('image_av')
	virtual_mirror_node = ImageAvNode()
	rospy.spin()
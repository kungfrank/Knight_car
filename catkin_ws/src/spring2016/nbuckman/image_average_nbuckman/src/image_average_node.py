#!/usr/bin/env python
import rospy
import cv2
import duckietown_msgs.msg
#from virtual_mirror_nbuckman import util
from std_msgs.msg import String,UInt8
import numpy as np
from sensor_msgs.msg import CompressedImage,Image
from cv_bridge import CvBridge, CvBridgeError


# Initialize the node with rospy

class ImageAverageNode(object):
	def __init__(self):
		self.node_name = "Image Average"
		self.sub_image = rospy.Subscriber("/ernie/camera_node/image/compressed", CompressedImage,self.avgImage)
		self.sub_avgimage = rospy.Subscriber("~average_image/compressed",CompressedImage)
		self.pub_avgimage = rospy.Publisher("~average_image/compressed", CompressedImage, queue_size=1)
		self.sub_time = rospy.Subscriber("~avg_time", UInt8)
		self.pub_time = rospy.Publisher("~avg_time", UInt8,queue_size=1)
		self.bridge = CvBridge()
	
	def avgImage(self,msg):
		np_array = np.fromstring(msg.data, np.uint8)
		image_np = cv2.imdecode(np_array, cv2.CV_LOAD_IMAGE_COLOR)
		
		np_array_avg = np.fromstring(self.suv_abgimage.data, np.uint8)		
		image_np_avg = cv2.imdecode(np_array_avg, cv2.CV_LOAD_IMAGE_COLOR)


		#Get number images before, publish increment
		new_time = self.sub_time+1
		self.pub_time(new_time)

		alpha_new = 1/new_time
		alpha_old = 1-alpha_new 
		avg_data = cv2.addWeighted(image_np,alpha_new,image_np_avg,alpha_old,0)
		avg_im = CompressedImage()
		avg_im.data = np.array(cv2.imencode('.jpg', avg_data)[1]).tostring()
		#flip_im.data = flip_arr.tostring()
		avg_im.header.stamp = msg.header.stamp
		self.pub_avgimage.publish(avg_im)



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
	rospy.init_node('image_average_node')
	virtual_mirror_node = ImageAverageNode()
	rospy.spin()
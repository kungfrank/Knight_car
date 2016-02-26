#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
import cv2
from cv_bridge import CvBridge, CvBridgeError




class image_average():
    def __init__(self):
	self.image_pub = rospy.Publisher("output",Image,queue_size=1)
	self.bridge = CvBridge()
	self.average_image = None
	self.sum_image = None
	self.acc=0;
    	rospy.Subscriber("input", CompressedImage, self.callback)

    def callback(self,original_image):
    	np_arr = np.fromstring(original_image.data, np.uint8)
	image_in = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
    	new_image = cv2.flip(image_in,1)
    	self.acc = self.acc+1
    	if self.average_image == None:
		self.average_image = new_image.copy()
		self.sum_image = new_image.copy()
    	else:
		self.average_image = cv2.addWeighted(self.average_image,(self.acc-1.0)/self.acc,image_in,1.0/self.acc,0)
    	self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.average_image,"bgr8"))

if __name__ == '__main__':
    rospy.init_node('image_average_node', anonymous=False)
    img_av = image_average()
    rospy.spin()



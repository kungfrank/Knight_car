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
    	self.acc = self.acc+1
    	if self.average_image == None or self.sum_image ==None:
	    self.average_image = image_in.copy()
	    self.sum_image = np.zeros((len(self.average_image),len(self.average_image[0]),len(self.average_image[0][0])))
        gain = 1.0/self.acc
    	for x in range(len(self.average_image)):
            for y in range(len(self.average_image[x])):
                for z in range(len(self.average_image[x][y])):
                    self.sum_image[x][y][z] =self.sum_image[x][y][z]+image_in[x][y][z]
                    #self.average_image[x][y][z] =((self.acc-1.0)/self.acc)*self.average_image[x][y][z] + (1.0/self.acc)*image_in[x][y][z]
                    self.average_image[x][y][z] =gain*self.sum_image[x][y][z]
	#self.average_image = cv2.addWeighted(self.average_image,(self.acc-1.0)/self.acc,image_in,1.0/self.acc,0)
        rospy.logwarn("%s",self.average_image[0][0])
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.average_image, "bgr8"))
        except CvBridgeError as e:
            rospy.warn("Exception while publishing image")
    	#self.image_pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('image_average_node', anonymous=False)
    img_av = image_average()
    rospy.spin()

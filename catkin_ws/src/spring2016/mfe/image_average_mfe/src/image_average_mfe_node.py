#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage,Image
import time
from copy import deepcopy

# bridge = CvBridge()
# publisher = None

class ImageAverageNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.bridge = CvBridge()
        
        #self.publish_freq = self.setupParam("~publish_freq",1.0)
        #self.publish_duration = rospy.Duration.from_sec(1.0/self.publish_freq)
        #self.pub_raw = rospy.Publisher("~image/raw",Image,queue_size=1)
	self.pub_ave = rospy.Publisher("~ave_img",Image,queue_size=1)
        self.last_stamp = rospy.Time.now()
        
        #self.sub_compressed_img = rospy.Subscriber("/ferrari/camera_node/image/compressed",CompressedImage,self.cbImg,queue_size=1)
	self.sub_raw_img = rospy.Subscriber("~raw_img",Image,self.cbImg,queue_size=1)
	self.average_image_data = np.empty(1)
	self.num_images_seen = 0.0	
    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbImg(self,raw_img_msg):
	print self.num_images_seen
	
	width = raw_img_msg.width
	height = raw_img_msg.height

	# This if statement should only run the first time through
	# since the size of the average image was unknown until
	# the first image arrives
	if len(self.average_image_data) != height*width*3:
		self.average_image_data = np.empty(width*height*3)
	# Convert raw image into numpy array, and combine with
	# stored average image information
	np_arr = np.fromstring(raw_img_msg.data,np.uint8)
	print 'First part of np_arr:',np_arr
	self.average_image_data = np.around((self.average_image_data*self.num_images_seen+np_arr)/(self.num_images_seen+1.0))
	self.num_images_seen += 1.0
	'''
	# To output ave_img_msg as a compressed image instead of raw:
	ave_img_msg = CompressedImage()
	ave_img_msg.header.stamp = rospy.Time.now()
	ave_img_msg.format = "jpeg"
	# the image has too many pixels for cv2.imencode...
	ave_img_msg.data = np.array(cv2.imencode('.jpg',self.average_image_data)[1]).tostring()
	'''
	ave_img_msg = deepcopy(raw_img_msg)
	ave_img_msg.data = self.average_image_data.astype(np.uint8).tostring()
	print 'First part of ave_img_msg.data',(ave_img_msg.data)[:20]
	print len(ave_img_msg.data),type(ave_img_msg.data)
	self.pub_ave.publish(ave_img_msg)

if __name__ == '__main__': 
    rospy.init_node('image_average',anonymous=False)
    node = ImageAverageNode()
    rospy.spin()

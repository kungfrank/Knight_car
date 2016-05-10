#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage,Image
import time
from copy import deepcopy
from duckietown_msgs.msg import Pixel
from duckietown_msgs_mfe.msg import FlipDirection
# bridge = CvBridge()
# publisher = None

class VirtualMirrorNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.bridge = CvBridge()
        
       	self.flip_direction_timer = rospy.Timer(rospy.Duration(1),self.cb_pub_flip_direction)
	self.pub_flip_direction = rospy.Publisher("~flip_direction",FlipDirection)
        #self.publish_duration = rospy.Duration.from_sec(1.0/self.publish_freq)
        self.pub_raw = rospy.Publisher("~image/mirrored",Image,queue_size=1)
        self.last_stamp = rospy.Time.now()        
        self.sub_raw_img = rospy.Subscriber("~raw_image",Image,self.cbImg,queue_size=1)
	
    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbImg(self,msg):
	print len(msg.data)
	width = msg.width
	height = msg.height
	mirrored_msg = deepcopy(msg)
	np_arr = np.fromstring(msg.data, np.uint8)
	new_arr = deepcopy(np_arr)
	for i in range(height):
		for j in range(width):
			for k in range(3):
				row_beg_ind = i*width*3
				pixel_ind = row_beg_ind+j*3
				mirror_ind = row_beg_ind+3*width-3*j
				#print row_beg_ind,pixel_ind,mirror_ind
				new_arr[k+pixel_ind] = np_arr[mirror_ind-3+k]
	print len(new_arr)
	mirrored_msg.data = new_arr.astype(np.uint8).tostring()
	self.pub_raw.publish(mirrored_msg)        
	'''
        ave_img_msg = deepcopy(raw_img_msg)
        ave_img_msg.data = self.average_image_data.astype(np.uint8).tostring()
        print 'First part of ave_img_msg.data',(ave_img_msg.data)[:20]
        print len(ave_img_msg.data),type(ave_img_msg.data)
        self.pub_ave.publish(ave_img_msg)
	'''

    def cb_pub_flip_direction(self,event):
	self.flip_direction = self.setupParam("~flip_direction",1.0)
	msg = FlipDirection()
	msg.header.stamp = rospy.Time.now()
	msg.flip_direction = self.flip_direction
	self.pub_flip_direction.publish(msg)
	
if __name__ == '__main__': 
    rospy.init_node('virtual_mirror_node',anonymous=False)
    node = VirtualMirrorNode()
    rospy.spin()
    
# will this show up on bitbucket? from github

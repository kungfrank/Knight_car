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

class DecoderNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.bridge = CvBridge()
        
        #self.publish_freq = self.setupParam("~publish_freq",1.0)
        #self.publish_duration = rospy.Duration.from_sec(1.0/self.publish_freq)
        self.pub_raw = rospy.Publisher("~image/mirrored",Image,queue_size=1)
        self.last_stamp = rospy.Time.now()        
        self.sub_compressed_img = rospy.Subscriber("/redrover/camera_node/image/raw",Image,self.cbImg,queue_size=1)
	
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
	mirrored_msg.data = new_arr.tolist()
	self.pub_raw.publish(mirrored_msg)
	'''
        if msg.header.stamp - self.last_stamp < self.publish_duration:
            return
        else:
            self.last_stamp = msg.header.stamp
	
        # time_start = time.time()
        np_arr = np.fromstring(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        # time_1 = time.time()
        img_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        # time_2 = time.time()
        img_msg.header.stamp = msg.header.stamp
        img_msg.header.frame_id = msg.header.frame_id
        self.pub_raw.publish(img_msg)

        # time_3 = time.time()
        # rospy.loginfo("[%s] Took %f sec to decompress."%(self.node_name,time_1 - time_start))
        # rospy.loginfo("[%s] Took %f sec to conver to Image."%(self.node_name,time_2 - time_1))
        # rospy.loginfo("[%s] Took %f sec to publish."%(self.node_name,time_3 - time_2))
	'''
if __name__ == '__main__': 
    rospy.init_node('decoder_low_freq',anonymous=False)
    node = DecoderNode()
    rospy.spin()

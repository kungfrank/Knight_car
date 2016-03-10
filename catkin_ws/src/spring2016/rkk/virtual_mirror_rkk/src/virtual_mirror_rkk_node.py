#!/usr/bin/env python
import sys
# numpy and scipy
import numpy as np
#from scipy.ndimage import filters

# OpenCV
import cv2
from cv_bridge import CvBridge, CvBridgeError

#ROS Libraries
#import roslib
import rospy

#ROS Messages
#from std_msgs.msg import String #Imports msg
from sensor_msgs.msg import CompressedImage, Image

#needed for timer:
#import time
#needed for ?
#import io

#import parameter
from duckietown_msg_rkk.msg import FlipDirection

class VirtualMirrorNode(object):
    def __init__(self):
        #parse node name
        self.node_name = rospy.get_name()
        #Create subscriber
        self.sub_comp = rospy.Subscriber("~rgb_in",CompressedImage,self.cbImg,queue_size=1)
        #Create publisher
        self.pub_raw = rospy.Publisher("~rgb_out",Image,queue_size=1)
            #self.pub_comp = rospy.Publisher("~rgb_out",CompressedImage,queue_size=1)        
        #setup dewfault parameter values
        self.flip_direction = self.setupParam("~flip_direction", "horz")
        #setup parameter publisher
        self.param_pub = rospy.Publisher("~flip_direction", FlipDirection, queue_size=1)
        self.param_msg = FlipDirection()
        #setup parameter timer callback
        self.param_timer = rospy.Timer(rospy.Duration.from_sec(1.0),self.cbParamTimer)
        #create opencv bridge object
        self.bridge = CvBridge()
        rospy.loginfo("%s has started", self.node_name)

    def cbParamTimer(self,event):
        self.flip_direction = rospy.get_param("~flip_direction", "horz")
        #self.param_msg = FlipDirection.HORZ if self.flip_direction == 'horz' else FlipDirection.VERT
        self.param_pub.publish(self.param_msg)

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value


    # Define callback function for image flipping
    def cbImg(self, msg):
        #print 'received image of type: "%s"' % msg.format
        #compressedImage first gets converted into a numpy array
        np_arr = np.fromstring(msg.data, np.uint8)
        #decode the image into a raw cv2 image (numpy.ndarray)
        rgb_in = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        #parse parameter
        if self.flip_direction == "horz":
            param = 1
        elif self.flip_direction == "vert":
            param = 0
        else:
            raise Exception('bad parameter used!')
        rgb_out = cv2.flip(rgb_in,param)

        msg_out = self.bridge.cv2_to_imgmsg(rgb_out,"bgr8")
        msg_out.header.stamp = msg.header.stamp
        msg_out.header.frame_id = msg.header.frame_id
        
        ## Publish new image
        self.pub_raw.publish(msg_out)
    
        ##### Create CompressedImage - NOT WORKING YET ####
        #msg_out = CompressedImage()
        #msg_out.header.stamp = msg.header.stamp
          #msg_out.header.frame_id = msg.header.frame_id
        #msg_out.format = "jpeg"
        #msg.data = np.array(self.bridge.imencode('.jpg', rgb_out)[1]).tostring()
        
        ## Publish new image
        #self.pub_comp.publish(msg_out) 
        #print 'send image of type: "%s"' % msg_out.format

if __name__ == '__main__': 
    # Initialize the node with rospy
    rospy.init_node('virtual_mirror_rkk')
    node = VirtualMirrorNode()
    rospy.spin() #Keeps the script for exiting

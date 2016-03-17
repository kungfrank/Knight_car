#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage,Image
from duckietown_msgs_lapentab.msg import CameraDirection
import time

# bridge = CvBridge()
# publisher = None

class VirtualMirror(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.bridge = CvBridge()
        self.flip_direction  = self.setupParam("~flip_direction","horz") # default horz
        self.pub_img = rospy.Publisher("~image/flipped",Image,queue_size=1)
        self.last_stamp = rospy.Time.now()
        self.sub_img = rospy.Subscriber("~img_in",CompressedImage, self.flipImage)
        self.param_timer = rospy.Timer(rospy.Duration.from_sec(1.0),self.cbParamTimer)

    def cbParamTimer(self,event):
        self.flip_direction = rospy.get_param("~flip_direction", "horz")


    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def flipImage(self,msg):
        if self.flip_direction == "vert":
            int_direction = 0
        else:
            int_direction = 1 # Default horz
        np_arr = np.fromstring(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        img_msg = cv2.flip(cv_image,int_direction)
        img_msg = self.bridge.cv2_to_imgmsg(img_msg, "bgr8") 
        self.pub_img.publish(img_msg)

if __name__ == '__main__':
    rospy.init_node('virtual_mirror_lapentab',anonymous=False)
    node = VirtualMirror()
    rospy.spin()


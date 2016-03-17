#!/usr/bin/env python
import rospy
import cv2
import io
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
import time

class VirtualMirrorTester(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing......" %(self.node_name))
        # TODO: load parameters

        self.pub_img = rospy.Publisher("~image/raw",Image,queue_size=1)

        self.sub_image = rospy.Subscriber("~mirror_image", Image, self.cbImage, queue_size=1)

        
        self.has_published = False
        self.bridge = CvBridge()

        # TODO setup other parameters of the camera such as exposure and white balance etc

        # Setup timer
        self.timer_img_low = rospy.Timer(rospy.Duration.from_sec(1.0/self.framerate),self.cbTimer)
        rospy.loginfo("[%s] Initialized." %(self.node_name))

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

     def cbImage(self,image_msg):
        image_cv = cv2.imdecode(np.fromstring(image_msg.data, np.uint8), cv2.CV_LOAD_IMAGE_COLOR)

    def cbTimer(self,event):
        if not rospy.is_shutdown():
            self.pub_img.publish(image_msg)
            # Maybe for every 5 img_low, change the setting of the camera and capture a higher res img and publish.
        if not self.has_published:
            rospy.loginfo("[%s] Published the first image." %(self.node_name))
            self.has_published = True

    def onShutdown(self):
        rospy.loginfo("[%s] Shutdown." %(self.node_name))

if __name__ == '__main__': 
    rospy.init_node('virtual_mirror_tristan_tester_node',anonymous=False)
    virtual_mirror_tester_node = VirtualMirrorTester()
    rospy.on_shutdown(virtual_mirror_tester_node.onShutdown)
    rospy.spin()
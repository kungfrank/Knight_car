#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image, CompressedImage
from duckietown_msg_teddy.msg import FlipDirection
import numpy as np
from cv_bridge import CvBridge
import cv2

# Virtual Mirror Node
# Author: Teddy Ort
# Inputs: Image
# Outputs: Image - A reflection of the input image
# This node was created as part of the M02-RCDP module

class VirtualMirrorNode(object):
    def __init__(self):
        self.node_name = 'Virtual Mirror'
        self.flip_direction     = self.setupParam("~flip_direction", 'vert')
        self.sub = rospy.Subscriber("~image_in", CompressedImage, self.processImage)
        self.pub = rospy.Publisher("~image_out", Image, queue_size=1)
        self.param_pub = rospy.Publisher("~flip_direction", FlipDirection, queue_size=1)
        self.param_msg = FlipDirection()
        self.bridge = CvBridge()
        self.param_timer = rospy.Timer(rospy.Duration.from_sec(1.0),self.cbParamTimer)
        rospy.loginfo("%s has started", self.node_name)

    def processImage(self, image_msg):
        #rospy.loginfo('Image Received')
        image_cv = cv2.imdecode(np.fromstring(image_msg.data, np.uint8), cv2.CV_LOAD_IMAGE_COLOR)
        flipCode = 0 if self.flip_direction == 'vert' else 1
        cv2.flip(image_cv, flipCode, image_cv)
        image_msg_out = self.bridge.cv2_to_imgmsg(image_cv, "bgr8")
        image_msg_out.header.stamp = image_msg.header.stamp
        self.pub.publish(image_msg_out)

    def cbParamTimer(self, event):
        self.flip_direction = rospy.get_param("~flip_direction", 1.0)
        self.param_msg.flip_direction = FlipDirection.HORZ if self.flip_direction == 'horz' else FlipDirection.VERT
        self.param_pub.publish(self.param_msg)

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

if __name__ == '__main__':
    rospy.init_node('virtual_mirror', anonymous=False)
    virtual_mirror_node = VirtualMirrorNode()
    rospy.spin()


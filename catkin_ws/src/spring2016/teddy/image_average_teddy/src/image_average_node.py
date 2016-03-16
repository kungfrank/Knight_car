#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image, CompressedImage
import numpy as np
from cv_bridge import CvBridge
import cv2

# Virtual Mirror Node
# Author: Teddy Ort
# Inputs: Image
# Outputs: Image - A horizontal reflection of the input image
# This node was created as part of the M02-RCDP module

class VirtualMirrorNode(object):
    def __init__(self):
        self.node_name = 'Image Average'
        self.sub = rospy.Subscriber("~image_in", CompressedImage, self.processImage)
        self.pub = rospy.Publisher("~image_out", Image, queue_size=1)
        self.bridge = CvBridge()
        self.image_avg = None
        self.image_count = 0
        rospy.loginfo("%s has started", self.node_name)

    def processImage(self, image_msg):
        #rospy.loginfo('Image Received')
        np_img = np.fromstring(image_msg.data, np.uint8)
        image_cv = cv2.imdecode(np_img, cv2.CV_LOAD_IMAGE_COLOR)/255.0

        if(self.image_count > 0):
            self.image_count += 1
            self.image_avg=cv2.addWeighted(image_cv,1.0/self.image_count,self.image_avg,1.0-1.0/self.image_count,0.0)
        else:
            self.image_count = 1
            self.image_avg = image_cv

        image_msg_out = self.bridge.cv2_to_imgmsg(cv2.convertScaleAbs(self.image_avg*255.0), "bgr8")
        image_msg_out.header.stamp = image_msg.header.stamp
        self.pub.publish(image_msg_out)

if __name__ == '__main__':
    rospy.init_node('virtual_mirror', anonymous=False)
    virtual_mirror_node = VirtualMirrorNode()
    rospy.spin()


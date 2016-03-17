#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Image, CompressedImage

import cv2
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()
image_pub = rospy.Publisher("output",Image,queue_size=1)

def callback(original_image):
    np_arr = np.fromstring(original_image.data, np.uint8)
    image_in = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
    new_image = cv2.flip(image_in,1)
    image_pub.publish(bridge.cv2_to_imgmsg(new_image,"bgr8"))

def listener():
    rospy.init_node('virtual_mirror_node', anonymous=False)
    rospy.Subscriber("input", CompressedImage, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()

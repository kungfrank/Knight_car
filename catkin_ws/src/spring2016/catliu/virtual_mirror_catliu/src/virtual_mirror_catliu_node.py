#!/usr/bin/env python
import rospy

from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage,Image
import time

bridge = CvBridge()

# Initialize the node with rospy
rospy.init_node('virtual_mirror_catliu_node')

# Define Timer callback
def callback(msg):
    np_arr = np.fromstring(msg.data, np.uint8)
    cv_image = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
    flipped = cv2.flip(cv_image, 1)
    img_msg = bridge.cv2_to_imgmsg(flipped, "bgr8")
    img_msg.header.stamp = msg.header.stamp
    img_msg.header.frame_id = msg.header.frame_id

    publisher.publish(img_msg)

# Create publisher
publisher = rospy.Publisher("~image_mirrored",Image,queue_size=1)
# Create subscriber
subscriber = rospy.Subscriber("~image_compressed", CompressedImage, callback)

# Read parameter
pub_period = rospy.get_param("~pub_period",1.0)

# spin to keep the script for exiting
rospy.spin()

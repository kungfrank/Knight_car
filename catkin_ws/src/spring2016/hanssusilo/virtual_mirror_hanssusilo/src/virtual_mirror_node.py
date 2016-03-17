#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

# Define callback function
def callback(msg):
	imgcv = cv2.imdecode(np.fromstring(msg.data, np.uint8), cv2.CV_LOAD_IMAGE_COLOR)
	flippedcv = cv2.flip(imgcv,1)
	imgout = bridge.cv2_to_imgmsg(flippedcv, "bgr8")
	imgout.header.stamp = msg.header.stamp
	publisher.publish(imgout)

# Initialize the node with rospy
rospy.init_node('virtual_mirror_node')
# Create publisher
publisher = rospy.Publisher("~img_topic_out",Image,queue_size=1)
# Create subscriber
subscriber = rospy.Subscriber("~img_topic_in", CompressedImage, callback)
# Create bridge
bridge = CvBridge()
rospy.spin() #Keeps the script for exiting

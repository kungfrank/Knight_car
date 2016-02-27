#!/usr/bin/env python
import rospy

import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import cv2
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage


# Initialize the node with rospy
rospy.init_node('virtual_mirror_node')

bridge = CvBridge()

# Define Timer callback
def callback(ros_data):
	# Convert to CV2
	np_arr = np.fromstring(ros_data.data, np.uint8)
  	image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
	
	# Flip image
	flipped = cv2.flip(image_np,1)

	# Back to ROS format(?)
	msg = bridge.cv2_to_imgmsg(flipped,"bgr8")
	publisher.publish(msg)

# Create publisher
publisher = rospy.Publisher("~image_out", Image, queue_size=1)

# Create subscriber
subscriber = rospy.Subscriber("~image_in", CompressedImage, callback, queue_size=1)

# spin to keep the script for exiting
rospy.spin()
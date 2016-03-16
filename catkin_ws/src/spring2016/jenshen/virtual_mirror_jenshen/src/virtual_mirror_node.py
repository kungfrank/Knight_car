#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

# Define callback function
def callback(msg):
    np_arr = np.fromstring(msg.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)

    image_out = cv2.flip(image_np, 1)
    image_out = bridge.cv2_to_imgmsg(image_out, encoding="passthrough")
    publisher.publish(image_out)

# Initialize the node with rospy
rospy.init_node('virtual_mirror_node')

# Create publisher
publisher = rospy.Publisher("~topic_out", Image, queue_size=1)
# Create subscriber
subscriber = rospy.Subscriber("~topic_in", CompressedImage, callback)

rospy.spin() #Keeps the script for exiting

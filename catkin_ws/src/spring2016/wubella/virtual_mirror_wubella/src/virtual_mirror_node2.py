#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
# Initialize the node with rospy
rospy.init_node('virtual_mirror_node')
# Define Timer callback
def callback(msg):
    np_arr = np.fromstring(msg.data, np.uint8)
    cv_image = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
    # time_1 = time.time()
    img_msg = bridge.cv2_to_imgmsg(cv_image, "bgr8")
    # time_2 = time.time()
    img_msg.header.stamp = msg.header.stamp
    img_msg.header.frame_id = msg.header.frame_id
    publisher.publish(img_msg)

bridge = CvBridge()
# Create publisher
publisher = rospy.Publisher("~topic", Image,queue_size=1)
# Create subscriber
subscriber = rospy.Subscriber("~topic_in", Image, callback)
# spin to keep the script for exiting
rospy.spin()

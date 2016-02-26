#!/usr/bin/env python
import rospy
from std_msgs.msg import String #Imports msg

from sensor_msgs.msg import CompressedImage #Imports msg

import numpy as np
import cv2

# Define callback function
def cam_callback(msg):
	np_arr = np.fromstring(msg.data, np.uint8)
	
	# Decode 
	image_rgb = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)

	# Mirror image laterally
	image_rgb_rev =	image_rgb[:,::-1,:]
	
	msg = CompressedImage()
	msg.header.stamp = rospy.Time.now()
	msg.format = "jpeg"
	msg.data = np.array(cv2.imencode('.jpg', image_rgb_rev)[1]).tostring()
	publisher.publish(msg)
	rospy.loginfo("Re Published ")

# Initialize the node with rospy
rospy.init_node('virtual_mirror')
# Create publisher
publisher = rospy.Publisher("/mirrored_image/compressed",CompressedImage,queue_size=1)
# Create subscriber
subscriber = rospy.Subscriber("/maserati/camera_node/image/compressed", CompressedImage, cam_callback)

rospy.spin() #Keeps the script for exiting
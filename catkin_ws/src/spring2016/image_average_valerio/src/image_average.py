#!/usr/bin/env python
import rospy
from std_msgs.msg import String #Imports msg

from sensor_msgs.msg import CompressedImage #Imports msg

import numpy as np
import cv2

imgsum = np.array(0)
count = 0

# Define callback function
def cam_callback(msg):
	global count
	global imgsum

	np_arr = np.fromstring(msg.data, np.uint8)

	# Decode 
	if(count==0):
		imgsum = np.double(cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR))
	else:
		imgsum += cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
	
	count=count+1

	imgavg = 1.0*imgsum/count

	imgout = CompressedImage();
	imgout.header.stamp = rospy.Time.now()
	imgout.format = "jpeg"
	imgout.data = np.array(cv2.imencode('.jpg', imgavg)[1]).tostring()
	publisher.publish(imgout)

# Initialize the node with rospy
rospy.init_node('image_average')
# Create publisher
publisher = rospy.Publisher("/maserati/camera_node/image_average/compressed",CompressedImage,queue_size=1)
# Create subscriber
subscriber = rospy.Subscriber("/maserati/camera_node/image/compressed", CompressedImage, cam_callback)

rospy.spin() #Keeps the script for exiting
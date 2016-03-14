#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import CompressedImage, Image
from duckietown_msg_hanssusilo.msg import FlipDir
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

# Define callback function
def callback(msg):
	flip = rospy.get_param("/penguin/virtual_mirror_hanssusilo_node/flip_direction")
	if flip == 'vert':
		flipi = FlipDir.vert
	elif flip == "horz":
		flipi = FlipDir.horz
	#print flipi
	imgcv = bridge.imgmsg_to_cv2(msg, "bgr8")
	#cv2.imdecode(np.fromstring(msg.data, np.uint8), cv2.CV_LOAD_IMAGE_COLOR)
	flippedcv = cv2.flip(imgcv,flipi)
	imgout = bridge.cv2_to_imgmsg(flippedcv, "bgr8")
	imgout.header.stamp = msg.header.stamp
	publisher.publish(imgout)

def paramTimerCB(event):
	flip = rospy.get_param("/penguin/virtual_mirror_hanssusilo_node/flip_direction")
        msg = FlipDir()
	if flip == 'vert':
		flipi = FlipDir.vert
	elif flip == "horz":
		flipi = FlipDir.horz
	msg.flip_direction = flipi
	flip_dir_publisher.publish(msg)

# Initialize the node with rospy
rospy.init_node('virtual_mirror_hanssusilo_node')
# Create subscriber
subscriber = rospy.Subscriber("img_orig", Image, callback)
# Create publisher
publisher = rospy.Publisher("img_flipped",Image,queue_size=1)
# Create publisher
flip_dir_publisher = rospy.Publisher("~flip_dir",FlipDir,queue_size=1)
# Create bridge
bridge = CvBridge()
# initialize parameter 
flip = rospy.get_param("/penguin/virtual_mirror_hanssusilo_node/flip_direction")
# timer to publish param
if flip == 'vert':
	flipi = FlipDir.vert
elif flip == "horz":
	flipi = FlipDir.horz
param_timer = rospy.Timer(rospy.Duration.from_sec(1.0),paramTimerCB)

rospy.spin() #Keeps the script for exiting

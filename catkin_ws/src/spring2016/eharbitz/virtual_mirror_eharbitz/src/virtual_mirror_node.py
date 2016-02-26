#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import numpy as np
import cv2


# Initialize the node with rospy
rospy.init_node('virtual_mirror_node')
# Create publisher
publisher = rospy.Publisher("~rbg_out",Image,queue_size=1)
# Create subscriber
subscriber = rospy.subscriber("~rgb_in")
# Define Timer callback
def callback(event):
	msg = String()
	msg.data = "%s is %s!" %(util.getName(),util.getStatus())
	publisher.publish(msg)
# Read parameter
pub_period = rospy.get_param("~pub_period",1.0)
# Create timer
rospy.Timer(rospy.Duration.from_sec(pub_period),callback)
# spin to keep the script for exiting
rospy.spin()

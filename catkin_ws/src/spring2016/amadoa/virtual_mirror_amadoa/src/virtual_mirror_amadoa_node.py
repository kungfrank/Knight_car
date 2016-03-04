#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from numpy import array

def setupParam(param_name,default_value):
	value = rospy.get_param(param_name,default_value)
	rospy.set_param(param_name,value) #Write to parameter server for transparancy
	rospy.loginfo("[virtual_mirror_amadoa_node] %s = %s " %(param_name,value))
	return value

flip_direction = setupParam("~flip_direction", "horz") # "horz" or "vert"

def imageCallback(msg):
	#### direct conversion to CV2 ####
	np_arr = np.fromstring(msg.data, np.uint8)
	image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)

	flipCode = 1 if flip_direction == "horz" else 0
	image_np_out = cv2.flip(image_np, flipCode)


	#### Create CompressedIamge ####
	out = CompressedImage()
	out.header.stamp = rospy.Time.now()
	out.format = "jpeg"
	np_arr_out = cv2.imencode('.jpg', image_np_out)[1]
	out.data = np.array(np_arr_out).tostring()

	# Publish new image
	publisher.publish(out)


def mirror():
	global publisher

	# Initialize the node with rospy
	rospy.init_node("virtual_mirror_amadoa_node")

	rospy.loginfo("virtual_mirror_amadoa_node initialized")
	# Create subscriber
	subscriber = rospy.Subscriber("/amadobot/camera_node/image/compressed", CompressedImage, imageCallback)

	# Create publisher
	publisher = rospy.Publisher("/amadobot/virtual_mirror_amadoa_node/image/compressed",CompressedImage,queue_size=1)

	# spin to keep the script for exiting
	rospy.spin()

if __name__ == '__main__':
	mirror()

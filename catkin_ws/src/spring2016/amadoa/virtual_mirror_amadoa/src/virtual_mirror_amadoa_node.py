#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from numpy import array


def imageCallback(msg):
	# rospy.loginfo("imagaCallback called")

	#### direct conversion to CV2 ####
	np_arr = np.fromstring(msg.data, np.uint8)
	image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
	# print('image_np_dimensions: %s x %s x %s' % (len(image_np), len(image_np[0]), len(image_np[0][0])))

	# print image_np[0] # result [[35 120 108] [35 120 108] [35 120 108] ...

	image_np_out = array([row[::-1] for row in image_np])



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

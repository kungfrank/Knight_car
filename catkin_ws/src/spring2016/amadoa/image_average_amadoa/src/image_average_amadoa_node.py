#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from numpy import array
# from image_average_amadoa import util

imageSum = 0
count = float(0)
def imageCallback(msg):
	global imageSum, count, zero
	# rospy.loginfo("imagaCallback called")
	
	#### direct conversion to CV2 ####
	np_arr = np.fromstring(msg.data, np.uint8)
	image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
	image_float = image_np.astype(np.float);

	count += 1
	if count == 1:
		imageSum = image_float
		# imageSum = imageSum
	else:
		beta = 1/count
		alpha = 1 - beta
		# print "for count = %s, (alpha, beta) = (%s, %s)"%(count, alpha, beta)
		print "----"
		imageSum = cv2.addWeighted(imageSum, alpha, image_float, beta, 0)

	imageAverage = imageSum
	print "first imageSum pixel: (%s, %s, %s)"% tuple(imageSum[0][0])
		# print "averaged"


	image_np_out = array(imageAverage)

	#### Create CompressedIamge ####
	out = CompressedImage()
	out.header.stamp = rospy.Time.now()
	out.format = "jpeg"
	np_arr_out = cv2.imencode('.jpg', image_np_out)[1]
	out.data = np.array(np_arr_out).tostring()

	# Publish new image
	publisher.publish(out)

def average():
	global publisher

	# Initialize the node with rospy
	rospy.init_node("image_average_amadoa_node")

	rospy.loginfo("image_average_amadoa_node initialized")
	# Create subscriber
	subscriber = rospy.Subscriber("/ferrari/camera_node/image/compressed", CompressedImage, imageCallback)

	# Create publisher
	publisher = rospy.Publisher("/amadobot/image_average_amadoa_node/image/compressed",CompressedImage,queue_size=1)

	# spin to keep the script for exiting
	rospy.spin()

if __name__ == '__main__':
	average()

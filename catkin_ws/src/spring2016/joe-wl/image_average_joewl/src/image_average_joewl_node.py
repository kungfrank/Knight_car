#!/usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image
import numpy as np

class ImageAverage(object):
    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()
        
        rospy.loginfo("[%s] Initialzing." %(self.node_name))

	# Initialize CV bridge
	self.bridge = CvBridge()

	# Initialize state variables:
	self.image_avg = None
	self.image_count = 0

        # Setup publishers
        self.pub_image_out = rospy.Publisher("~image_avg", Image, queue_size=1)

        # Setup subscriber
        self.sub_image_in = rospy.Subscriber("~image_in", CompressedImage, self.avgImage)

        rospy.loginfo("[%s] Initialzed." %(self.node_name))

    def avgImage(self,image_msg):
	# Convert image message to CV image:
	#image_cv = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
	image_cv = cv2.imdecode(np.fromstring(image_msg.data, np.uint8), cv2.CV_LOAD_IMAGE_COLOR)

	# Incorporate image in running average (initialize if first image):
	self.image_count += 1
	if (self.image_avg != None):
		weight = 1.0/self.image_count
		self.image_avg = cv2.addWeighted(self.image_avg,(1-weight),image_cv,weight,0)
	else:
		self.image_avg = image_cv

	# Convert average image to image message:
	image_msg_out = self.bridge.cv2_to_imgmsg(self.image_avg, "bgr8")
	image_msg_out.header.stamp = image_msg.header.stamp

	# Publish average image:
	self.pub_image_out.publish(image_msg_out)

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('image_average_joewl', anonymous=False)

    # Create the NodeName object
    image_average_joewl_node = ImageAverage()

    # Setup proper shutdown behavior 
    rospy.on_shutdown(image_average_joewl_node.on_shutdown)
    
    # Keep it spinning to keep the node alive
    rospy.spin()

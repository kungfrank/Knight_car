#!/usr/bin/env python
import rospy
import cv2
import sys
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image
import numpy as np
import threading


class averaged_image:

	def __init__(self):
		self.node_name = "image_average_npd22"

		#initialize bridge
		self.bridge = CvBridge()

		#'''Initialize ros publisher, ros subscriber'''
	       	# topic where we publish
	    	self.image_pub = rospy.Publisher("~image_averaged", Image, queue_size=1)

	      	# subscribed Topic
	      	self.subscriber = rospy.Subscriber("/ferrari/camera_node/image/compressed", CompressedImage, self.average,  queue_size = 1)
		rospy.loginfo("[%s] Initialized." %(self.node_name))

		#initialize image variable and 
		self.image_out = np.zeros((640,480,3), np.float32)
		self.image_count = 0

	def average(self, image_msg):

		#### direct conversion to CV2 ####
	       	image_cv = cv2.imdecode(np.fromstring(image_msg.data, np.uint8), cv2.CV_LOAD_IMAGE_COLOR)


		
		self.image_count = self.image_count + 1
		if self.image_count > 1:
			self.image_out = cv2.addWeighted(self.image_out,((float(self.image_count)-1.0)/float(self.image_count)),image_cv.astype('float32'),(1.0/float(self.image_count)),0)

		else:
			self.image_out = image_cv.astype('float32')

		image_uint8 = self.image_out.astype('uint8')

		# Publish the image message
        	image_msg_out = self.bridge.cv2_to_imgmsg(image_uint8, "bgr8")
        	image_msg_out.header.stamp = image_msg.header.stamp	

		self.image_pub.publish(image_msg_out)

		# Decode from compressed image
       	 	# with OpenCV
        	#image_cv = cv2.imdecode(np.fromstring(image_msg.data, np.uint8), cv2.CV_LOAD_IMAGE_COLOR)

		#cv_image = bridge.imgmsg_to_cv2(image_message, desired_encoding="passthrough")

	def onShutdown(self):
            rospy.loginfo("[mirror] Shutdown.")   	

#def main(args):
	#'''Initializes and cleanup ros node'''
#	ic = image_feature()
	# Initialize the node with rospy
#	rospy.init_node('virtual_mirror_npd22')
#	try:
#		rospy.spin()
#	except KeyboardInterrupt:
#		print "Shutting down ROS Image feature detector module"
#	cv2.destroyAllWindows()


if __name__ == '__main__':
    rospy.init_node('image_average_npd22',anonymous=False)
    image_average_node = averaged_image()
    rospy.on_shutdown(averaged_image.onShutdown)
    rospy.spin()

#class VirtualMirrorNpd22(object):
#    def __init__(self):
#        self.node_name = "virtual_mirror_npd22"

	# Publishers
#        self.pub_image = rospy.Publisher("~image_mirrored", CompressedImage, queue_size=1)

        # Subscribers
#	subscriber = rospy.Subscriber("~topic", String, callback)


#        self.sub_image = rospy.Subscriber("~image", CompressedImage, self.cbImage, queue_size=1)
#        # rospy.loginfo("[%s] Initialized." %(self.node_name))

#    def processImage(self,image_msg):
#	# Decode from compressed image
#        # with OpenCV
#        image_cv = cv2.imdecode(np.fromstring(image_msg.data, np.uint8), cv2.CV_LOAD_IMAGE_COLOR)

	# Flip image
	# with OpenCV
#	image_mirrored = cv2.flip(image_msg.data, dst=None, flipMode=1)

	# Publish mirrored image
 #       self.pub_image.publish(image_mirrored)

# Read parameter
#pub_period = rospy.get_param("~pub_period",1.0)
# Create timer
#rospy.Timer(rospy.Duration.from_sec(pub_period),callback)
# spin to keep the script for exiting
#rospy.spin()

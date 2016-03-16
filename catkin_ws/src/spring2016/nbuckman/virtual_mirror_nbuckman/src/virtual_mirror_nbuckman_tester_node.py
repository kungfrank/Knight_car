#!/usr/bin/env python
import rospy
import cv2
import duckietown_msgs.msg
from duckietown_msg_nbuckman.msg import Flip
#from virtual_mirror_nbuckman import util
from std_msgs.msg import String
import numpy as np
from sensor_msgs.msg import CompressedImage,Image
from cv_bridge import CvBridge, CvBridgeError
import time


# Initialize the node with rospy

class VirtualMirrorNbuckmanTesterNode(object):
	def __init__(self):
		self.node_name = "Virtual Mirror Nbuckman Tester"
		self.fake_pub_image = rospy.Publisher("/ernie/camera_node/image/compressed", CompressedImage,queue_size=1)
		img_file = rospy.get_param("~img_file","/home/noambuckman/test_images/01_horz.jpg")
		rospy.loginfo(img_file)
		im_cv = cv2.imread(img_file)
		img_str = cv2.imencode('.jpg', im_cv)[1].tostring()		
		pub_im=CompressedImage()
		pub_im.header=rospy.Time.now()
		pub_im.data = img_str
		#pub_im.data = np.array(cv2.imencode('.jpg', im_np)[1]).tostring()
		#flip_im.data = flip_arr.tostring()
		#flip_im.header.stamp = msg.header.stamp
		self.fake_pub_image.publish(pub_im)

if __name__ == '__main__': 
	rospy.init_node('virtual_mirror_nbuckman_tester')
	virtual_mirror_node = VirtualMirrorNbuckmanTesterNode()
	rospy.spin()
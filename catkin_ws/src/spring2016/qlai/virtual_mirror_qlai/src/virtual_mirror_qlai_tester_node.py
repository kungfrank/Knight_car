#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage,Image
import time
import io


class VirtualMirrorTesterNode(object):
	def __init__(self):
		self.node_name = "Virtual Mirror Tester"
		self.bridge = CvBridge()

		self.flip = rospy.get_param("~flip_direction")
		self.img = rospy.get_param("~test_img")
		self.pub_comp = rospy.Publisher("~img_out", CompressedImage, queue_size=1)
		self.sub_raw = rospy.Subscriber("~img_in", Image, self.compareImg, queue_size=1 )
		self.runTest()


	def getTestImg(self):
		self.original = cv2.imencode("test_images/"+self.img+"_orig.png")
		self.flipped = cv2.imread("test_images/"+self.img+"_"+self.flip".png")
		rospy.loginfo("imgs loaded successfully")

	def runTest(self):
		self.getTestImg()
		imgmsg = self.bridge.cv2_to_imgmsg(self.original, "bgr8")
		self.pub_comp.publish(imgmsg)

	def compareImg(self, msg):
		np_arr = np_



	def onShutdown(self):
		    rospy.loginfo("[VirtualMirrorTesterNode] Shutdown.")

if __name__ == '__main__':
    rospy.init_node('virtual_mirror_qlai_tester',anonymous=False)
    virtual_mirror_qlai_tester_node = VirtualMirrorTesterNode()
    rospy.on_shutdown(virtual_mirror_qlai_tester_node.onShutdown)




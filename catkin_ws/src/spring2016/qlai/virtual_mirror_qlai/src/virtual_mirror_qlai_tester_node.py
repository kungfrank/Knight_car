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
		

		self.horzp = rospy.get_param("~fliphorz")
		self.vertp = rospy.get_param("~flipvert")

		self.runTest()



	def getTestImg(self):
		self.originals = [] 
		self.originals.append(cv2.imread("test_images/01_orig.png"))
		self.originals.append(cv2.imread("test_images/02_orig.png"))
		self.originals.append(cv2.imread("test_images/03_orig.png"))
		self.originals.append(cv2.imread("test_images/04_orig.png"))


		self.horz = []
		self.horz.append(cv2.imread("test_images/01_horz.png"))
		self.horz.append(cv2.imread("test_images/02_horz.png"))
		self.horz.append(cv2.imread("test_images/03_horz.png"))
		self.horz.append(cv2.imread("test_images/04_horz.png"))


		self.vert = []
		self.vert.append(cv2.imread("test_images/01_vert.png"))
		self.vert.append(cv2.imread("test_images/02_vert.png"))
		self.vert.append(cv2.imread("test_images/03_vert.png"))
		self.vert.append(cv2.imread("test_images/04_vert.png"))

		rospy.loginfo("imgs loaded successfully")

	def runTest(self):
		self.getTestImg()
		for i in range(4):
			if self.horz[i] == cv2.flip(self.originals[i], self.horzp):
				rospy.loginfo("image has been horizontally flipped correctly")
			else:
				rospy.loginfo("ERROR in image horizontal flip")

			if self.vert[i] == cv2.flip(self.originals[i], self.vertp):
				rospy.loginfo("image has been vertically flipped correctly")
			else:
				rospy.loginfo("ERROR in image vertical flip")

	def onShutdown(self):
		    rospy.loginfo("[VirtualMirrorTesterNode] Shutdown.")

if __name__ == '__main__':
    rospy.init_node('virtual_mirror_qlai_tester',anonymous=False)
    virtual_mirror_qlai_tester_node = VirtualMirrorTesterNode()
    rospy.on_shutdown(virtual_mirror_qlai_tester_node.onShutdown)




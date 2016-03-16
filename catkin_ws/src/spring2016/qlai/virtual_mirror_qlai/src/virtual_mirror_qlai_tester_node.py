#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage,Image
from duckietown_msg_qlai.msg import FlipDirection
import time
import io


class VirtualMirrorTesterNode(object):
	def __init__(self):
		self.node_name = "Virtual Mirror Tester"
		self.bridge = CvBridge()
		self.vert = False
		self.horz = False
		self.flip = "horz"
		# self.flip = rospy.get_param("~flip_direction")
		self.img = str(rospy.get_param("~test_img"))
		self.pub_comp = rospy.Publisher("virtual_mirror_qlai_node/rgb_in", CompressedImage, queue_size=1, latch = True)
		self.sub_flip = rospy.Subscriber("virtual_mirror_qlai_node/flip_direction", FlipDirection, self.checkFlip)
		self.sub_raw = rospy.Subscriber("virtual_mirror_qlai_node/rgb_out", Image, self.compareImg, queue_size=1)
		
		rospy.loginfo("Initialized.")
		self.original = cv2.imread("../test_images/0"+self.img+"_orig.png")
		# self.check()
		# self.compareImg()
	

	def getTestImg(self):
		rospy.loginfo("img loaded successfully")
		# rospy.loginfo(self.original)
		self.imgmsg = CompressedImage()
		self.imgmsg.header.stamp = rospy.Time.now()
		self.imgmsg.format = "png"
		self.imgmsg.data = np.array(cv2.imencode('.png', self.original)[1]).tostring()
		self.pub_comp.publish(self.imgmsg)
		rospy.loginfo("msg from test published")

	def checkFlip(self, flip_dir):
		self.flip = flip_dir.direction
		rospy.loginfo("got flip direction msg = "+self.flip)
		self.check()


	def check(self):
		rospy.loginfo("checking")
		if self.vert == True and self.horz == True:
			rospy.loginfo("tests complete")
		else:
			self.getTestImg()
		if self.horz == False:
			if self.flip != "horz":
				# self.flip = "horz"
				rospy.set_param("virtual_mirror_qlai_node/flip_direction", "horz")
		elif self.vert == False:
			if self.flip != "vert":
				# self.flip = "vert"
				rospy.set_param("virtual_mirror_qlai_node/flip_direction", "vert")


	def compareImg(self, msg):
		rospy.loginfo("comparing imgs")
		image_cv = self.bridge.imgmsg_to_cv2(msg, "bgr8")

		flippedimg = cv2.imread("../test_images/0"+self.img+"_"+self.flip+".png")
		# print flippedimg
		# print image_cv
		if np.array_equal(flippedimg, image_cv):
			rospy.loginfo("success")
		else:
			rospy.loginfo("failed")


		if self.flip == "horz":
			self.horz = True

		if self.flip == "vert":
			self.vert = True


	def onShutdown(self):
		    rospy.loginfo("[VirtualMirrorTesterNode] Shutdown.")

if __name__ == '__main__':
    rospy.init_node('virtual_mirror_qlai_tester',anonymous=False)
    virtual_mirror_qlai_tester_node = VirtualMirrorTesterNode()
    rospy.on_shutdown(virtual_mirror_qlai_tester_node.onShutdown)
    rospy.spin()




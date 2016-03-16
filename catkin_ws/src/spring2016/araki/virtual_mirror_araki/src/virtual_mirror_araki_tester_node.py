#!/usr/bin/env python
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
import time
import io
from duckietown_msg_araki.msg import FlipDirection

class image_flipper:

    def __init__(self):
        self.node_name = rospy.get_name()
        self.bridge = CvBridge()

        self.vert = False
        self.horz = False
        self.flip = "horz"

        self.img = str(rospy.get_param("~test_img"))

        self.pub_comp = rospy.Publisher("virtual_mirror_araki_node/img_in",CompressedImage,queue_size=1, latch = True)
        self.sub_flip = rospy.Subscriber("virtual_mirror_araki_node/flip_direction",FlipDirection,self.checkFlip)
        self.sub_raw = rospy.Subscriber("virtual_mirror_araki_node/img_out",Image,self.compareImg,queue_size=1)

        self.original = cv2.imread("../test_images/0"+self.img+"_orig.png")

    def getTestImg(self):
        rospy.loginfo("img loaded")
        self.imgmsg = CompressedImage()
        self.imgmsg.header.stamp = rospy.Time.now()
        self.imgmsg.format = "png"
        self.imgmsg.data = np.array(cv2.imencode('.png',self.original)[1]).tostring()
        self.pub_comp.publish(self.imgmsg)
        rospy.loginfo("test msg published")

    def checkFlip(self, flip_dir):
        self.flip = flip_dir.flip_direction
        rospy.loginfo("got flip direction msg = "+self.flip)
        self.check()

    def check(self):
        if self.vert == True and self.horz == True:
            rospy.loginfo("tests complete")
        else:
            self.getTestImg()
        if self.horz == False:
            if self.flip != "horz":
                rospy.set_param("virtual_mirror_araki_node/flip_direction", "horz")
        elif self.vert == False:
            if self.flip != "vert":
                rospy.set_param("virtual_mirror_araki_node/flip_direction", "vert")

    def compareImg(self,msg):
        rospy.loginfo("comparing imgs")
        image_cv = self.bridge.imgmsg_to_cv2(msg,"bgr8")

        flippedimg = cv2.imread("../test_images/0"+self.img+"_"+self.flip+".png")

        if np.array_equal(flippedimg,image_cv):
            rospy.loginfo("success")
        else:
            rospy.loginfo("failed")


   
def virtual_mirror_araki_node():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('virtual_mirror_araki_node', anonymous=True)
    flipper = image_flipper()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
    cv2.destroyAllWindows()


if __name__ == '__main__':
    virtual_mirror_araki_node()
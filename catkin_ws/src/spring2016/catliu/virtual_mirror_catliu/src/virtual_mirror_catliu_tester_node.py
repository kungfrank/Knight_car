#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from duckietown_catliu_msgs.msg import MirrorOrientation
from sensor_msgs.msg import CompressedImage,Image

class VirtualMirrorCatliuTesterNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        

        self.pub_fake_images = rospy.Publisher("virtual_mirror_catliu/image_compressed", CompressedImage, queue_size=1)
        rospy.sleep(1)

        #take in directory of images as input, use 01_orig.png, 02, 03, 04
        self.im_dir = self.setupParam("~im_dir", "")
        self.num_im = self.setupParam("~num_im", 0)
        self.num_sent = 0
        self.orientation = None
        self.checkedHorz = False
        self.checkedVert = False

        self.bridge = CvBridge()
        self.sub_mirrored = rospy.Subscriber("virtual_mirror_catliu/image_mirrored", Image, self.checkIm)
        self.sub_mirrored = rospy.Subscriber("virtual_mirror_catliu/orientation", MirrorOrientation, self.checkOrientation)

    def setupParam(self, param_name, default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def checkIm(self, im):
        image = self.bridge.img_msg_to_cv2(im, "bgr8")
        checkfilename = ''
        if self.orientation==MirrorOrientation.VERT:
            #compare im to 0N_vert.png
            checkfilename = '0'+self.num_sent+'_vert.png'
            self.checkedVert = True
        else:
            checkfilename = '0'+self.num_sent+'_horz.png'
            self.checkedHorz = True
        flipped = cv2.imread(self.im_dir+checkfilename)
        if np.array_equal(image, flipped):
            rospy.loginfo("[%s] Image %s %s passed" %(self.node_name,self.sent_image_count,self.orientation))
        else:
            rospy.loginfo("[%s] Image %s %s failed" %(self.node_name,self.sent_image_count,self.orientation))

    def checkOrientation(self, orientation):
        #compare against 0N_vert.png, 0N_horz.png
        self.orientation = orientation.orientation
        if self.checkedHorz and self.checkedVert and self.num_sent<self.num_im:
            self.checkedHorz = False
            self.checkedVert = False
            self.num_sent+=1
            #publish next image to check horizontal mirror
            rospy.set_param("flip_direction", 'horz')
            self.pubImage()
        elif self.orientation == MirrorOrientation.VERT and not self.checkedHorz:
            #Switch to checking horizontal again
            rospy.set_param("flip_direction", 'horz')
            self.pubImage()
        elif self.orientation == MirrorOrientation.HORZ and self.checkedHorz and not self.checkedVert:
            #publish next image to check vertically
            rospy.set_param("flip_direction", 'vert')
            self.pubImage()

    def pubImage(self):
        filename = '0'+self.num_sent+'_orig.png'
        image = cv2.imread(self.im_dir+filename)
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = 'png'
        msg.data = np.array(cv2.imencode('.png', image)[1]).tostring()
        self.pub_fake_images(msg)

    def onShutdown(self):
        rospy.loginfo("[VirtualMirrorCatliuTesterNode] Shutdown.")

if __name__ == '__main__':
    rospy.init_node('virtual_mirror_catliu_tester',anonymous=False)
    virtual_mirror_catliu_tester = VirtualMirrorCatliuTesterNode()
    rospy.on_shutdown(virtual_mirror_catliu_tester.onShutdown)

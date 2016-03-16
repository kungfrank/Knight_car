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
        

        self.pub_fake_images = rospy.Publisher("virtual_mirror_catliu_node/image_compressed", CompressedImage, queue_size=1)
        rospy.sleep(1)

        #take in directory of images as input, use 01_orig.png, 02, 03, 04
        self.im_dir = self.setupParam("~im_dir", "")
        self.num_im = self.setupParam("~num_im", 0)
        self.orientation = None
        self.checkedHorz = False
        self.checkedVert = False

        self.bridge = CvBridge()
        self.sub_mirrored = rospy.Subscriber("virtual_mirror_catliu_node/image_mirrored", Image, self.checkIm)
        self.sub_mirrored = rospy.Subscriber("virtual_mirror_catliu_node/orientation", MirrorOrientation, self.checkOrientation)
        #start the first image
        self.num_sent = 1
        rospy.sleep(1)
        self.pubImage()

    def setupParam(self, param_name, default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def checkIm(self, im):
        image = self.bridge.imgmsg_to_cv2(im, "bgr8")
        checkfilename = ''
        if self.orientation==MirrorOrientation.VERT:
            #compare im to 0N_vert.png
            checkfilename = '0'+str(self.num_sent)+'_vert.png'
            self.checkedVert = True
        else:
            checkfilename = '0'+str(self.num_sent)+'_horz.png'
            self.checkedHorz = True
        flipped = cv2.imread(self.im_dir+checkfilename)
        if np.array_equal(image, flipped):
            rospy.loginfo("[%s] Image %s %s passed" %(self.node_name,self.num_sent,self.orientation))
        else:
            rospy.loginfo("[%s] Image %s %s failed" %(self.node_name,self.num_sent,self.orientation))

    def checkOrientation(self, orientation):
        #compare against 0N_vert.png, 0N_horz.png
        self.orientation = orientation.orientation
        #rospy.loginfo("Checking orientation %s, checkedHorz %s, checkedVert %s" %(self.orientation, self.checkedHorz, self.checkedVert))
        if self.checkedHorz and self.checkedVert and self.num_sent<self.num_im:
            self.checkedHorz = False
            self.checkedVert = False
            self.num_sent+=1
            #publish next image to check horizontal mirror
            rospy.set_param("virtual_mirror_catliu_node/flip_direction", 'horz')
            self.orientation = MirrorOrientation.HORZ
            rospy.sleep(1)
            self.pubImage()
        elif self.orientation == MirrorOrientation.VERT and not self.checkedHorz:
            #Switch to checking horizontal again
            rospy.set_param("virtual_mirror_catliu_node/flip_direction", 'horz')
            self.orientation = MirrorOrientation.HORZ
            rospy.sleep(1)
            self.pubImage()
        elif self.orientation == MirrorOrientation.HORZ and self.checkedHorz and not self.checkedVert:
            #publish next image to check vertically
            rospy.set_param("virtual_mirror_catliu_node/flip_direction", 'vert')
            self.orientation = MirrorOrientation.VERT
            rospy.sleep(1)
            self.pubImage()

    def pubImage(self):
        filename = '0'+str(self.num_sent)+'_orig.png'
        image = cv2.imread(self.im_dir+filename)
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = 'png'
        msg.data = np.array(cv2.imencode('.png', image)[1]).tostring()
        self.pub_fake_images.publish(msg)
        rospy.loginfo("[%s] Image %s published" %(self.node_name,filename))

    def onShutdown(self):
        rospy.loginfo("[VirtualMirrorCatliuTesterNode] Shutdown.")

if __name__ == '__main__':
    rospy.init_node('virtual_mirror_catliu_tester',anonymous=False)
    virtual_mirror_catliu_tester = VirtualMirrorCatliuTesterNode()
    rospy.on_shutdown(virtual_mirror_catliu_tester.onShutdown)
    rospy.spin()
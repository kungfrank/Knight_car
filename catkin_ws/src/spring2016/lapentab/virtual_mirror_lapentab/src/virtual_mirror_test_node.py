#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage,Image
from duckietown_msgs_lapentab.msg import CameraDirection
import time
import random
import os

# bridge = CvBridge()
# publisher = None

class VirtualMirrorTest(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.bridge = CvBridge()
        self.flip_direction  = self.setupParam("/magitek/virtual_mirror_node/flip_direction","horz") # default horz
        self.pub_img = rospy.Publisher("~image_orig",CompressedImage,queue_size=1)
        self.last_stamp = rospy.Time.now()
        self.sub_img = rospy.Subscriber("~img_in", Image, self.compareImage)
        self.param_timer = rospy.Timer(rospy.Duration.from_sec(1.0),self.cbParamTimer)
        self.pub_timer = rospy.Timer(rospy.Duration.from_sec(1.0),self.pubTimer)
        test_img = random.choice([1,2,3])
        test_img = str(test_img)
        cur_path = os.path.dirname(os.path.realpath(__file__))
        self.img_orig = cv2.imread(cur_path + '/test_images/0' +test_img+'_orig.png')
        self.img_vert = cv2.imread(cur_path + '/test_images/0' +test_img+'_vert.png')
        self.img_horz = cv2.imread(cur_path + '/test_images/0' +test_img+'_horz.png')
        self.img_vert_comp = np.asarray(self.img_vert, dtype=np.uint8)
        self.img_horz_comp = np.asarray(self.img_horz, dtype=np.uint8)
        rospy.loginfo(cur_path + '/test_images/0' +test_img+'_orig.png')

        
    def cbParamTimer(self,event):
        self.flip_direction = rospy.get_param("/magitek/virtual_mirror_node/flip_direction")    

    def pubTimer(self,event):
        img_orig_compressed = CompressedImage()
        img_orig_compressed.format = "png"
        img_orig_compressed.data=np.array(cv2.imencode('.png', self.img_orig)[1]).tostring()
        img_orig_compressed.header.stamp = rospy.Time.now()
        self.pub_img.publish(img_orig_compressed)  


    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def compareImage(self,msg):
        img_cv = self.bridge.imgmsg_to_cv2(msg)
        in_image = np.asarray(img_cv, dtype=np.uint8)
        if self.flip_direction == "vert":
            if (in_image == self.img_vert_comp).all():
                rospy.loginfo("Pass")
                rospy.set_param("/magitek/virtual_mirror_node/flip_direction","horz")
            else:
                rospy.loginfo("Fail")
        else:
            if (in_image == self.img_horz_comp).all():
                rospy.loginfo("Pass")
                rospy.set_param("/magitek/virtual_mirror_node/flip_direction","vert")
            else:
                rospy.loginfo("Fail")

if __name__ == '__main__':
    rospy.init_node('virtual_mirror_lapentab',anonymous=False)
    node = VirtualMirrorTest()
    rospy.spin()


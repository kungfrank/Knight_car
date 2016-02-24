#!/usr/bin/env python
import cv2
import numpy as np
from matplotlib import pyplot as plt
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge, CvBridgeError
import sys
import os


template = cv2.imread('blob.png')
(h, w, _)= template.shape
template_small = cv2.resize(template, (w/2, h/2))
# All the 6 methods for comparison in a list
meth = 'cv2.TM_SQDIFF_NORMED'

def match(img):
    img = cv2.cvtColor(img, 1)
    method = eval(meth)
    # Apply template Matching
    res = cv2.matchTemplate(img,template,method)
    res_small = cv2.matchTemplate(img,template_small,method)
    min_val, _ , top_left, _  = cv2.minMaxLoc(res)
    min_val_small, _ , top_left_small, _  = cv2.minMaxLoc(res_small)
    temp_w = w
    temp_h = h
    if min_val_small < min_val:
        min_val = min_val_small
        top_left = top_left_small
        temp_w = w/2
        temp_h =h/2

    
    bottom_right = (top_left[0] + temp_w, top_left[1] + temp_h)
    if min_val < .3:
        cv2.rectangle(img,top_left, bottom_right, 255, 2)
        pose = .5 -( (img.shape[1]-(top_left[0] + .5*temp_h)) / img.shape[1] )
    else:
        pose = float('nan')
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    return img, pose


class ConeDetector:
    def __init__(self, target_img="cone.png"):
        self.node_name = "Cone Detector"
        template = cv2.imread('blob.png',0)
        w, h = template.shape[::-1]
        self.sub_image = rospy.Subscriber("/camera/rgb/image_rect_color", \
                Image, self.processImage)
        self.pub_image = rospy.Publisher("~cone_detection", Image, queue_size=1)
        self.pub_pose = rospy.Publisher("~cone_ibvs", Float32, queue_size=1)
        self.bridge = CvBridge()

    def processImage(self, image_msg):
        #self.sub_image.unregister()
        image_cv = self.bridge.imgmsg_to_cv2(image_msg)
        img,pose = match(image_cv)
        try:
            self.pub_image.publish(self.bridge.cv2_to_imgmsg(img, "rgb8"))
        except CvBridgeError as e:
            print(e)
        pub_pose = Float32()
        pub_pose.data = pose
        self.pub_pose.publish(pub_pose)
        
        """
        # Resize and crop image
        hei_original = image_cv.shape[0]
        wid_original = image_cv.shape[1]
        if self.hei_image!=hei_original or self.wid_image!=wid_original:
            image_cv = cv2.resize(image_cv, (self.wid_image, self.hei_image))
        image_cv = image_cv[self.top_cutoff:,:,:]

        # Set the image to be detected
        self.detector.setImage(image_cv)
	
        """
if __name__=="__main__":
    rospy.init_node('cone_detector')
    c = ConeDetector()
    rospy.spin()


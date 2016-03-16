#!/usr/bin/env python
import cv2
import numpy as np
from matplotlib import pyplot as plt
import rospy
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float32
from cv_bridge import CvBridge, CvBridgeError
import sys
import os
import threading


class VirtualMirror:
    def __init__(self):
        self.node_name = "Virtual_Mirror"
        self.thread_lock = threading.Lock()
        #self.pub_image_mirror = rospy.Publisher("~virtual_mirror_img", Image, queue_size=1)
        #self.pub_image_regular = rospy.Publisher("~virtual_regular_img", Image, queue_size=1)
        self.pub_image_average = rospy.Publisher("~virtual_regular_img_avg", Image, queue_size=1)
        self.bridge = CvBridge()
        self.image_avg = None
        self.t = 0
        self.sub_image = rospy.Subscriber("~image_raw", CompressedImage, self.cbImage, queue_size=1)

# Subscribers
        rospy.loginfo("[%s] Initialized." %(self.node_name))

    def cbImage(self,image_msg):
        # Start a daemon thread to process the image
        thread = threading.Thread(target=self.processImage,args=(image_msg,))
        thread.setDaemon(True)
        thread.start()
        # Returns rightaway


    def processImage(self, image_msg):
        if not self.thread_lock.acquire(False):
            # Return immediately if the thread is locked
            return
        #self.sub_image.unregister()
        np_arr = np.fromstring(image_msg.data, np.uint8)
        image_cv = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
         
        self.t +=1
        if self.t == 1:
            self.image_avg = image_cv
        else:
            prev_w = (self.t-1.0)/self.t
            curr_w = 1.0/self.t
            weight_avg = lambda (c,p) :int( prev_w*p + curr_w*c)

            # do processing
            (H,W,_) = image_cv.shape

            H -= 1
            W -= 1
        
            for u in range(H):
                for v in range(W):
                    curr = list(image_cv[u][v])
                    prev = list(self.image_avg[u][v])
                    avg = [weight_avg ( (curr[i], prev[i]) ) for i in range(3)] 
                    self.image_avg[u][v] = avg
        try:
            self.pub_image_average.publish(self.bridge.cv2_to_imgmsg(self.image_avg, "bgr8"))
        except CvBridgeError as e:
            print(e)
        self.t +=1
        self.thread_lock.release()
        """
        (H,W,_) = image_cv.shape

        H -= 1
        W -= 1
    
        for u in range(H):
            for v in range(W):
               # img[u][v] = image_cv[u, W-v]

                if self.image_prev !=None:
                    curr = list(image_cv[u][v])
                    prev = list(self.image_prev[u][v])
                    avg = [ (curr[i] + prev[i])/2 for i in range(3)] 
                    img_avg[u][v] = avg
        try:
            #self.pub_image_mirror.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
            #self.pub_image_regular.publish(self.bridge.cv2_to_imgmsg(image_cv, "bgr8"))

            if self.image_prev != None:
                self.pub_image_average.publish(self.bridge.cv2_to_imgmsg(img_avg, "bgr8"))

        except CvBridgeError as e:
            print(e)
        self.image_prev = image_cv
        self.thread_lock.release()

        """

if __name__=="__main__":
    rospy.init_node('virtual_mirror')
    c = VirtualMirror()
    rospy.spin()


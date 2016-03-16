#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage,Image
import time

# bridge = CvBridge()
# publisher = None

class BlurImage(object):
    """ """
    def __init__(self):    
        """ """
        self.node_name = rospy.get_name()
        self.bridge = CvBridge()
        
        self.average_exist = False
        self.n             = 0
        self.alpha         = 0.05
        
        self.pub_blur           = rospy.Publisher( "image/blur", Image, queue_size=1)
        self.sub_compressed_img = rospy.Subscriber( "camera_node/image/compressed" , CompressedImage, self.callback, queue_size=1)

    def callback(self,msg):
        """ """ 
        
        # Load message
        np_arr = np.fromstring(msg.data, np.uint8)
        cv_img = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)   
        
        self.n = self.n + 1
        #print self.n
        
        # Average
        if self.average_exist:
            
            # Sum then divide
            #self.sum = self.sum + cv_img
            #average  =  self.sum * 1./self.n 
            #self.cv_img_average = np.around(average).astype(np.uint8)
            
            # Direct update
            self.cv_img_average   = cv2.addWeighted( cv_img , self.alpha , self.cv_img_average , ( 1. - self.alpha ) , 0 )

        else:
            # Init  
            print "Init"
            self.cv_img_average = cv_img.copy()
            self.sum            = cv_img.copy().astype(float )
            self.average_exist  = True
        
        
        # Publish Message
        img_msg = self.bridge.cv2_to_imgmsg( self.cv_img_average , "bgr8")
        img_msg.header.stamp = msg.header.stamp
        img_msg.header.frame_id = msg.header.frame_id
        self.pub_blur.publish(img_msg)


if __name__ == '__main__': 
    rospy.init_node('blur_image',anonymous=False)
    node = BlurImage()
    rospy.spin()
#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage,Image
import time

# bridge = CvBridge()
# publisher = None

class AprilPrePros(object):
    """ """
    def __init__(self):    
        """ """
        self.node_name = rospy.get_name()
        self.bridge = CvBridge()
 
        self.pub_ToApril_global = rospy.Publisher( "apriltags_global/image_raw", Image, queue_size=1)
        self.pub_ToApril_fast   = rospy.Publisher( "apriltags_fast/image_raw", Image, queue_size=1)
        self.sub_compressed_img = rospy.Subscriber( "camera_node/image/compressed" , CompressedImage, self.callback, queue_size=1)
        
        self.global_timer = rospy.Timer(rospy.Duration.from_sec(2.0), self.global_detection )
        
        self.fast_timer   = rospy.Timer(rospy.Duration.from_sec(0.250), self.fast_detection )
        
        self.camera_IMG  = None
        self.camera_msg  = None
        
        

    def callback(self,msg):
        """ Process camera IMG """ 
        
        # Load message
        #cv_img = self.bridge.imgmsg_to_cv2( msg.data )
        np_arr = np.fromstring(msg.data, np.uint8)
        cv_img = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        
        self.camera_IMG  = cv_img
        self.camera_msg  = msg
        
        
    def global_detection(self,event):
        """ Pre-pros image and publish """
        
        if not self.camera_IMG == None:
        
            # Crop
            a = 100
            c = -50
            b = 50
            #crop_img = self.camera_IMG[0+a+c:480-a+c, 0+b:640-b]
            crop_img = self.camera_IMG
            
            # Downsample
            h,w = crop_img.shape[:2]
            print h,w
            processed_img = cv2.pyrDown(crop_img,dstsize = (w/2,h/2))
            #processed_img = crop_img
    
            # Publish Message
            img_msg = self.bridge.cv2_to_imgmsg( processed_img , "bgr8")
            img_msg.header.stamp = self.camera_msg.header.stamp
            img_msg.header.frame_id = self.camera_msg.header.frame_id
            self.pub_ToApril_global.publish(img_msg)
            
        else:
            print 'No camera image to process'
            
        
            
            
    def fast_detection(self,event):
        """ Pre-pros image and publish """
        
        if not self.camera_IMG == None:
        
            # Crop
            a = 100
            c = -50
            b = 50
            #crop_img = self.camera_IMG[0+a+c:480-a+c, 0+b:640-b]
            crop_img = self.camera_IMG
            
            # Downsample
            h,w = crop_img.shape[:2]
            print h,w
            processed_img = cv2.pyrDown(crop_img,dstsize = (w/2,h/2))
            #processed_img = crop_img
    
            # Publish Message
            img_msg = self.bridge.cv2_to_imgmsg( processed_img , "bgr8")
            img_msg.header.stamp = self.camera_msg.header.stamp
            img_msg.header.frame_id = self.camera_msg.header.frame_id
            self.pub_ToApril_fast.publish(img_msg)
            
        else:
            print 'No camera image to process'


if __name__ == '__main__': 
    rospy.init_node('AprilPrePros',anonymous=False)
    node = AprilPrePros()
    rospy.spin()
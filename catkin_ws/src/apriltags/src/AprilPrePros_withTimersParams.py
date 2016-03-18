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
        rospy.loginfo("[%s] Initializing " %(self.node_name))
        self.bridge = CvBridge()
 
        self.pub_ToApril_global = rospy.Publisher( "apriltags_global/image_raw", Image, queue_size=1)
        self.pub_ToApril_fast   = rospy.Publisher( "apriltags_fast/image_raw", Image, queue_size=1)
        self.sub_compressed_img = rospy.Subscriber( "camera_node/image/compressed" , CompressedImage, self.callback, queue_size=1)
        
        self.param_timer  = rospy.Timer(rospy.Duration.from_sec(1.0),    self.load_params  )
        
        self.camera_IMG  = None
        self.camera_msg  = None
        
        self.load_params( None )
        self.init_timers()
        
        
        
    def load_params(self, event):
        """ """
        
        #Timers period
        self.fast_period   = rospy.get_param("~fast_period")
        self.global_period = rospy.get_param("~global_period")
        
        # Cropping Factors
        self.fast_h_crop     = rospy.get_param("~fast_h_crop")
        self.fast_v_crop     = rospy.get_param("~fast_v_crop")
        self.fast_v_off      = rospy.get_param("~fast_v_off")
        self.global_h_crop   = rospy.get_param("~global_h_crop")
        self.global_v_crop   = rospy.get_param("~global_v_crop")
        self.global_v_off    = rospy.get_param("~global_v_off")
        
        rospy.loginfo("[%s] Parameters Loaded " %(self.node_name))
        
        
    def init_timers(self):
        """ """
        
        self.global_timer = rospy.Timer(rospy.Duration.from_sec( self.global_period ), self.global_detection )
        self.fast_timer   = rospy.Timer(rospy.Duration.from_sec( self.fast_period ), self.fast_detection )
        

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
            a = self.global_v_crop # up/down edge crop
            c = self.global_v_off # horizontal offset
            b = self.global_h_crop  # Side crop
            
            crop_img = self.camera_IMG[0+a+c:480-a+c, 0+b:640-b]
            #crop_img = self.camera_IMG
            
            # Downsample
            h,w = crop_img.shape[:2]
            #processed_img = cv2.pyrDown(crop_img,dstsize = (w/2,h/2))
            processed_img = crop_img
    
            # Publish Message
            img_msg = self.bridge.cv2_to_imgmsg( processed_img , "bgr8")
            img_msg.header.stamp = self.camera_msg.header.stamp
            img_msg.header.frame_id = self.camera_msg.header.frame_id
            self.pub_ToApril_global.publish(img_msg)
            
            rospy.loginfo("[%s] Global Detection Processed " %(self.node_name))
            
        else:
            
            rospy.loginfo("[%s] Global Detection: No camera image to process " %(self.node_name))
            
        
            
            
    def fast_detection(self,event):
        """ Pre-pros image and publish """
        
        if not self.camera_IMG == None:
        
            # Crop
            a = self.fast_v_crop # up/down edge crop
            c = self.fast_v_off # horizontal offset
            b = self.fast_h_crop  # Side crop
            
            crop_img = self.camera_IMG[0+a+c:480-a+c, 0+b:640-b]
            #crop_img = self.camera_IMG
            
            # Downsample
            h,w = crop_img.shape[:2]
            #processed_img = cv2.pyrDown(crop_img,dstsize = (w/2,h/2))
            processed_img = crop_img
    
            # Publish Message
            img_msg = self.bridge.cv2_to_imgmsg( processed_img , "bgr8")
            img_msg.header.stamp = self.camera_msg.header.stamp
            img_msg.header.frame_id = self.camera_msg.header.frame_id
            self.pub_ToApril_fast.publish(img_msg)
            
            rospy.loginfo("[%s] Fast Detection Published " %(self.node_name))
            
        else:
            
            rospy.loginfo("[%s] Fast Detection: No camera image to process " %(self.node_name))


if __name__ == '__main__': 
    rospy.init_node('AprilPrePros',anonymous=False)
    node = AprilPrePros()
    rospy.spin()
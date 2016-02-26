#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage,Image
import time

# bridge = CvBridge()
# publisher = None

class Mirror(object):
    """ """
    def __init__(self):    
        """ """
        self.node_name = rospy.get_name()
        self.bridge = CvBridge()
 
        self.pub_mirror         = rospy.Publisher( "image/mirror", Image, queue_size=1)
        self.sub_compressed_img = rospy.Subscriber( "camera_node/image/compressed" , CompressedImage, self.callback, queue_size=1)

    def callback(self,msg):
        """ """ 
        
        # Load message
        #cv_img = self.bridge.imgmsg_to_cv2( msg.data )
        np_arr = np.fromstring(msg.data, np.uint8)
        cv_img = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        
        # Mirror
        cv_img_mirror = cv_img.copy()
        cv_img_mirror = cv2.flip(cv_img,1)      
            
        # Publish Message
        img_msg = self.bridge.cv2_to_imgmsg( cv_img_mirror , "bgr8")
        img_msg.header.stamp = msg.header.stamp
        img_msg.header.frame_id = msg.header.frame_id
        self.pub_mirror.publish(img_msg)


if __name__ == '__main__': 
    rospy.init_node('mirror',anonymous=False)
    node = Mirror()
    rospy.spin()
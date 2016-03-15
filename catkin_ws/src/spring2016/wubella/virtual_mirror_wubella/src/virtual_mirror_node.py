#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage,Image
import time
from duckietown_msg_wubella.msg import FlipDirection

# bridge = CvBridge()
# publisher = None

class VirtualMirror(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.bridge = CvBridge()
        
        #self.publish_freq = self.setupParam("~publish_freq",1.0)
        #self.publish_duration = rospy.Duration.from_sec(1.0/self.publish_freq)
        self.pub_raw = rospy.Publisher("~image/compressed", CompressedImage,queue_size=1)
        #self.last_stamp = rospy.Time.now()        
        self.sub_compressed_img = rospy.Subscriber("~image_in",CompressedImage,self.cbImg, queue_size=1)
        
        self.flip_direction = FlipDirection()
        self.flip_direction.direction = self.flip_direction.horz#default horizontal mirror
        self.param_pub = rospy.Publisher("~flip_direction", FlipDirection, queue_size=1)
        self.param_pub_timer = rospy.Timer(rospy.Duration.from_sec(1.0),self.cbPubTimer)
        self.param_sub_timer = rospy.Timer(rospy.Duration.from_sec(1.0),self.cbSubTimer)

    def cbPubTimer(self,event):
        self.param_pub.publish(self.flip_direction)

    def cbSubTimer(self,event):
        direction_string = rospy.get_param("~flip_direction", 'horz')
        if direction_string == self.flip_direction.vert or direction_string == self.flip_direction.horz:
            self.flip_direction.direction = direction_string
        rospy.loginfo(self.flip_direction.direction)

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbImg(self,msg):
        # time_start = time.time()

        np_arr = np.fromstring(msg.data, np.uint8)
         
        cv_image = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        #cv_image = cv2.flip(cv_image, 1)

        if self.flip_direction.direction == self.flip_direction.horz:
            cv_image = cv2.flip(cv_image, 1)
        if self.flip_direction.direction == self.flip_direction.vert:
            cv_image = cv2.flip(cv_image, 0)
        
        compressed_img_msg = CompressedImage()
        compressed_img_msg.format = "png"
       	compressed_img_msg.data = np.array(cv2.imencode('.png', cv_image)[1]).tostring()

        compressed_img_msg.header.stamp = msg.header.stamp
        compressed_img_msg.header.frame_id = msg.header.frame_id
        self.pub_raw.publish(compressed_img_msg)

if __name__ == '__main__': 
    rospy.init_node('virtual_mirror_wubella',anonymous=False)
    node = VirtualMirror()
    rospy.spin()


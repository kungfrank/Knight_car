#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage,Image
from duckietown_msg_qlai.msg import FlipDirection
import time
import io
# bridge = CvBridge()
# publisher = None

class VirtualMirrorNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.bridge = CvBridge()
        
        # self.publish_freq = self.setupParam("~publish_freq",1.0)
        # self.publish_duration = rospy.Duration.from_sec(1.0/self.publish_freq)
        
        # self.last_stamp = rospy.Time.now()        
        self.flipdirection = self.setupParam("~flip_direction", "horz")
        self.pub_raw = rospy.Publisher("~rgb_out",Image,queue_size=1)
        self.param_timer = rospy.Timer(rospy.Duration.from_sec(1.0),self.cbParamTimer)
        self.pub_timer = rospy.Timer(rospy.Duration.from_sec(1.0), self.cbPubTimer)
        self.pub_flip = rospy.Publisher("~flip_direction", FlipDirection, queue_size=1)
        self.sub_compressed_img = rospy.Subscriber("~rgb_in",CompressedImage,self.cbImg,queue_size=1)
        rospy.loginfo("virtual mirror init")
        # self.stream = io.BytesIO()
        

    def cbImg(self, msg):
        rospy.loginfo("in cbImg virtual mirror")
        np_arr = np.fromstring(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        # time_1 = time.time()
        if self.flipdirection == "horz":
            mirror_image = cv2.flip(cv_image, 1)
        elif self.flipdirection == "vert":
            mirror_image = cv2.flip(cv_image, 0)
        else:
            raise Exception('invalid parameter')
        
        img_msg = self.bridge.cv2_to_imgmsg(mirror_image, "bgr8")
        # time_2 = time.time()
        img_msg.header.stamp = msg.header.stamp
        img_msg.header.frame_id = msg.header.frame_id
        self.pub_raw.publish(img_msg)

    def cbParamTimer(self,event):
        # rospy.loginfo("in cbTimer virtual mirror")
        self.flipdirection = self.setupParam("~flip_direction", "horz")

    def cbPubTimer(self, event):
        # rospy.loginfo("in timer pub virtual mirror")
        flip_msg = FlipDirection()
        flip_msg.direction = self.flipdirection
        self.pub_flip.publish(flip_msg)
        # rospy.loginfo(flip_msg)

        
    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        # rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

if __name__ == '__main__': 
    rospy.init_node('mirrorimg',anonymous=False)
    node = VirtualMirrorNode()
    rospy.spin()



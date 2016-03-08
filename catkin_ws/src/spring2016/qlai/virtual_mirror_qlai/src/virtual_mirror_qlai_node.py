#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage,Image
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
        self.pub_raw = rospy.Publisher("~rgb_out",Image,queue_size=1)
        # self.last_stamp = rospy.Time.now()        
        self.flipdirection = self.setupParam("~flip_direction", "horz")
        self.param_timer = rospy.Timer(rospy.Duration.from_sec(1.0),self.cbParamTimer)        
        self.sub_compressed_img = rospy.Subscriber("~rgb_in",CompressedImage,self.cbImg,queue_size=1)
        # self.stream = io.BytesIO()
        

    def cbImg(self, msg):
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
        self.flipdirection = self.setupParam("~flip_direction", "horz")

        
    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

if __name__ == '__main__': 
    rospy.init_node('mirrorimg',anonymous=False)
    node = VirtualMirrorNode()
    rospy.spin()



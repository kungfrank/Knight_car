#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage,Image
import time

# bridge = CvBridge()
# publisher = None

class VirtualMirror(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.bridge = CvBridge()

        self.pub_img = rospy.Publisher("~image/flipped",Image,queue_size=1)
        self.last_stamp = rospy.Time.now()
        self.sub_img = rospy.Subscriber("~img_in",CompressedImage, self.flipImage)

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def flipImage(self,msg):
        np_arr = np.fromstring(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        # time_1 = time.time()
        img_msg = cv2.flip(cv_image,1)
        # time_2 = time.time()
	img_msg = self.bridge.cv2_to_imgmsg(img_msg, "bgr8") 
        self.pub_img.publish(img_msg)

        # time_3 = time.time()
        # rospy.loginfo("[%s] Took %f sec to decompress."%(self.node_name,time_1 - time_start))
        # rospy.loginfo("[%s] Took %f sec to conver to Image."%(self.node_name,time_2 - time_1))
        # rospy.loginfo("[%s] Took %f sec to publish."%(self.node_name,time_3 - time_2))

if __name__ == '__main__':
    rospy.init_node('virtual_mirror_lapentab',anonymous=False)
    node = VirtualMirror()
    rospy.spin()


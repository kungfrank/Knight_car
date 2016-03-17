#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
# from sensor_msgs.msg import CompressedImage,Image
from sensor_msgs.msg import Image
import time
import IPython

class VirtualMirrorNode(object):
    def invImg(self,msg):
        print('.')
        # if msg.header.stamp - self.last_stamp < self.publish_duration:
        #     return
        # else:
        #     self.last_stamp = msg.header.stamp
        # time_start = time.time()
        np_arr = np.fromstring(msg.data, np.uint8)
        # IPython.embed()
        src_img=self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        cv_image = cv2.flip(src_img, 1)
        # time_1 = time.time()
        img_msg = self.bridge.cv2_to_imgmsg(cv_image, "rgb8")
        # time_2 = time.time()
        img_msg.header.stamp = msg.header.stamp
        img_msg.header.frame_id = msg.header.frame_id
        img_msg.width=msg.width;
        img_msg.height=msg.height;
        self.pub_raw.publish(img_msg)

    def __init__(self):
        self.node_name = rospy.get_name()
        self.bridge = CvBridge()
        self.pub_raw = rospy.Publisher("~inverted_image",Image,queue_size=1)
        self.sub_compressed_img = rospy.Subscriber("~topic_in",Image,self.invImg,queue_size=1)


if __name__ == '__main__': 
    rospy.init_node('decoder_low_freq',anonymous=False)
    node = VirtualMirrorNode()
    rospy.spin()

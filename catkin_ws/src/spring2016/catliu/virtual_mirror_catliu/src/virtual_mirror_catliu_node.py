#!/usr/bin/env python
import rospy

from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage,Image
import time
from duckietown_catliu_msgs.msg import MirrorOrientation

class VirtualMirrorCatliuNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.flip_direction = self.setupParam("~flip_direction",'horz')
        if self.flip_direction == 'vert':
            self.flip_direction = MirrorOrientation()
            self.flip_direction.orientation = MirrorOrientation.VERT
        else:
            self.flip_direction = MirrorOrientation()
            self.flip_direction.orientation = MirrorOrientation.HORZ

        # Create publisher
        self.im_publisher = rospy.Publisher("~image_mirrored",Image,queue_size=1)
        # Create subscriber
        self.im_subscriber = rospy.Subscriber("~image_compressed", CompressedImage, self.callback)
        #Listen for changes to orientation
        self.timer_trim = rospy.Timer(rospy.Duration.from_sec(1.0),self.cbOrientation)
        #Create publisher for orientation
        self.orientation_pub = rospy.Publisher("~orientation",MirrorOrientation, queue_size=1)
        # timer
        self.pub_timer = rospy.Timer(rospy.Duration.from_sec(1.0),self.publishOrientation)

    def publishOrientation(self, event):
        pub_msg = MirrorOrientation()
        pub_msg.orientation = self.flip_direction.orientation
        self.orientation_pub.publish(pub_msg)

    def setupParam(self, param_name, default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value
    
    def cbOrientation(self,event):
        orientation = rospy.get_param("~flip_direction",'horz')
        if orientation == 'vert':
            orientation = MirrorOrientation.VERT
        else:
            orientation = MirrorOrientation.HORZ
        if self.flip_direction.orientation != orientation:
            self.flip_direction.orientation = orientation
            rospy.loginfo("[%s] Orientation updated to: %s"%(self.node_name,orientation))
    
    # Define Timer callback
    def callback(self, msg):
        bridge = CvBridge()
        np_arr = np.fromstring(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        flipped = cv2.flip(cv_image, self.flip_direction.orientation)
        img_msg = bridge.cv2_to_imgmsg(flipped, "bgr8")
        img_msg.header.stamp = msg.header.stamp
        img_msg.header.frame_id = msg.header.frame_id

        self.im_publisher.publish(img_msg)

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('virtual_mirror_catliu_node')
    virtual_mirror_catliu_node = VirtualMirrorCatliuNode()

    # spin to keep the script for exiting
    rospy.spin()
